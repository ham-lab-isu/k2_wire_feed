///*******************************************************************************
/**
    K2-MultiThreaded

	The main function for Multi-Threaded ClearPath-SC example. The only command
	line argument is the port number where the network is attached. This
	main function opens the port, prints some basic information about the
	nodes that are found, checks that they are all in full-access mode,
	then creates the Supervisor object (which has its own thread) to run
	the show. This main thread then waits for the user to hit a key to
	end the program.
**/
//******************************************************************************

#include <stdio.h>
#include <ctime>
#include <chrono>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include "k2_action/action/wire_feed.hpp"
#include "pubSysCls.h"
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <array>

using namespace sFnd;
using namespace std;
//#include "Axis.h"
//#include "Supervisor.h"

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
}

// Check if the bus power has been applied.
bool IsBusPowerLow(INode &theNode) {
    return theNode.Status.Power.Value().fld.InBusLoss;
}

/*****************************************************************************
*  Function Name: AttentionDetected
*	 Description:	This is the port-level attention handler function.
*					This handler simply prints out the attention information
*					to the console window. 
*	  Parameters:
*          Input:	detected		- contains the attention information
*         Return:		none
*****************************************************************************/
void MN_DECL AttentionDetected(const mnAttnReqReg &detected)
{
	// Make a local, non-const copy for printing purposes
	mnAttnReqReg myAttns = detected;
	// Create a buffer to hold the attentionReg information
	char attnStringBuf[512];
	// Load the buffer with the string representation of the attention information
	myAttns.AttentionReg.StateStr(attnStringBuf, 512);
	// Print it out to the console
	printf("  --> ATTENTION: port %d, node=%d, attn=%s\n",
		detected.MultiAddr >> 4, detected.MultiAddr, attnStringBuf);
}

#if _MSC_VER
#pragma warning(disable:4996)
#endif
// A nice way of printing out the system time
string CurrentTimeStr() {
	time_t now = time(NULL);
	return string(ctime(&now));
}
#define CURRENT_TIME_STR CurrentTimeStr().c_str()
#if _MSC_VER
#pragma warning(default:4996)
#endif

#define MAX_MOTORS 16
#define ACC_LIM_RPM_PER_SEC	100000
#define VEL_LIM_RPM			700
#define MOVE_DISTANCE_CNTS	10000	
#define NUM_MOVES			5
#define TIME_TILL_TIMEOUT	10000

/************* Wire Feed Server Class Definition *********** */

using WireFeed = k2_action::action::WireFeed;

class WireFeedDistanceServer : public rclcpp::Node {
	public:		
		// Constructor: build the action server and bind the major functions: handle_goal, handle_cancel, and handle_accepted
		WireFeedDistanceServer() : Node("wire_feed_distance_server") {
			distance_server_ = rclcpp_action::create_server<WireFeed>(
				this,
				"wire_feed_distance",
				std::bind(&WireFeedDistanceServer::handle_distance_goal, this, std::placeholders::_1, std::placeholders::_2),
				std::bind(&WireFeedDistanceServer::handle_distance_cancel, this, std::placeholders::_1),
				std::bind(&WireFeedDistanceServer::handle_distance_accepted, this, std::placeholders::_1)
			);

			RCLCPP_INFO(this->get_logger(), "WireFeed Action Server is up and running!");

			// Initialize the motor axes
			initialize_axes();

			// Initialize execution threads for each axis
			for (int i = 0; i < MAX_MOTORS; ++i) {
				execution_threads_[i] = std::thread(&WireFeedDistanceServer::process_goals, this, i);
			}		
		}

		~WireFeedDistanceServer() {
			running_ = false; // Stop execution loops
			for (int i = 0; i < MAX_MOTORS; ++i) {
				goal_condition_[i].notify_all();
				if (execution_threads_[i].joinable()) {
					execution_threads_[i].join();
				}
			}
			quit_supervisor();
		}

	private:
		// Action server
		rclcpp_action::Server<WireFeed>::SharedPtr distance_server_;

		// Queue for each motor
		std::array<std::queue<std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>>>, MAX_MOTORS> goal_queues_;
		std::array<std::mutex, MAX_MOTORS> queue_mutex_;
		std::array<std::condition_variable, MAX_MOTORS> goal_condition_;
		std::array<std::thread, MAX_MOTORS> execution_threads_;
		std::atomic<bool> running_{true};std::queue<std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>>> goal_queue_;

		// Teknic members
		std::vector<INode*> listOfNodes;
		SysManager* myMgr;
		int COUNTS_PER_REV;

		float64 roller_diameter = 26.035; //mm - wire roller diameter
		size_t portCount = 0;

		std::vector<std::string> comHubPorts;

		std::thread goal_thread;

		// Assume that the nodes are of the right type and that this app has full control
		bool nodeTypesGood = true, accessLvlsGood = true; 

		// Method to initialize the SC Hubs as ports and motors as nodes
		void initialize_axes(){
			RCLCPP_INFO(this->get_logger(), "Initializing axes...");

			//Create the SysManager object. This object will coordinate actions among various ports
			// and within nodes. In this example we use this object to setup and open our port.
			myMgr = SysManager::Instance();

			//This will try to open the port. If there is an error/exception during the port opening,
			//the code will jump to the catch loop where detailed information regarding the error will be displayed;
			//otherwise the catch loop is skipped over
			SysManager::FindComHubPorts(comHubPorts);

			for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
				myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
												// with COM portnum (as seen in device manager)
			}

			if (portCount > 0) {
				//printf("\n I will now open port \t%i \n \n", portnum);
				myMgr->PortsOpen(portCount);				//Open the port

				for (size_t i = 0; i < portCount; i++) {
					IPort &myPort = myMgr->Ports(i);

					RCLCPP_INFO(this->get_logger(), " Port[%d]: state=%d, nodes=%d\n",
						myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
				}
			}
			else {
				printf("Unable to locate SC hub port\n");

				//msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
			}

			for (size_t iPort = 0; iPort < portCount; iPort++){
				// Get a reference to the port, to make accessing it easier
				IPort &myPort = myMgr->Ports(iPort);

				// Enable the attentions for this port
				myPort.Adv.Attn.Enable(true);
				// The attentions will be handled by the individual nodes, but register
				// a handler at the port level, just for illustrative purposes.
				myPort.Adv.Attn.AttnHandler(AttentionDetected);
				double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable

				for (unsigned iNode = 0; iNode < myPort.NodeCount(); iNode++){

					// Get a reference to the node, to make accessing it easier
					INode &theNode = myPort.Nodes(iNode);
					theNode.EnableReq(false);

					// Set the node parameters
					theNode.Motion.AccLimit = 100000;
					theNode.Status.AlertsClear();					//Clear Alerts on node 
					theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
					theNode.EnableReq(true);					//Enable node 

					// Make sure we are talking to a ClearPath SC
					if (theNode.Info.NodeType() != IInfo::CLEARPATH_SC_ADV) {
						printf("---> ERROR: Uh-oh! Node %d is not a ClearPath-SC Advanced Motor\n", iNode);
						this->nodeTypesGood = false;
					}

					if (this->nodeTypesGood) {
						// add node to list
						this->listOfNodes.push_back(&theNode);
						
						if (!theNode.Setup.AccessLevelIsFull()) {
							printf("---> ERROR: Oh snap! Access level is not good for node %u\n", iNode);
							this->accessLvlsGood = false;
						}
					}

					while (!theNode.Motion.IsReady()) {
						if (myMgr->TimeStampMsec() > timeout) {
							if (IsBusPowerLow(theNode)) {
								printf("Error: Bus Power low. Make sure 75V supply is powered on.\n");
								msgUser("Press any key to continue.");
								return;
							}
							printf("Error: Timed out waiting for Node %d to enable\n", iNode);
							msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
							return;
						}
					}

					// If we have full access to the nodes and they are all ClearPath-SC advanced nodes, 
					// then continue with the example
					if (nodeTypesGood && accessLvlsGood){
						printf("\nMachine starting: %s\n", CURRENT_TIME_STR);
					}
					else{
						// If something is wrong with the nodes, tell the user about it and quit
						if (!nodeTypesGood){
							printf("\n\tFAILURE: Please attach only ClearPath-SC Advanced nodes.\n\n");
						}
						else if (!accessLvlsGood){
							printf("\n\tFAILURE: Please get full access on all your nodes.\n\n");
						}
					}
				}
			}
		}

		// Goal Callback
		rclcpp_action::GoalResponse handle_distance_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const WireFeed::Goal> goal) {
			RCLCPP_INFO(this->get_logger(), "Received goal request.");
			(void)uuid;
			(void)goal;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		
		// Cancel Callback
		rclcpp_action::CancelResponse handle_distance_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
			RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
			(void)goal_handle;

			// get the node object
			auto goal = goal_handle->get_goal();
        	int axis = goal->axis_number;
			//INode &theNode = myPort.Nodes(axis);

			// call the Node Stop method; clear it immediately
			//theNode.Motion.NodeStop(0x11);
			//theNode.Motion.NodeStopClear();
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		// Goal Accepted Callback
		void handle_distance_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
			RCLCPP_INFO(this->get_logger(), "Received new goal request, adding to queue.");
	        auto goal = goal_handle->get_goal();
        	int axis = goal->axis_number;

			if (axis < 0 || axis >= MAX_MOTORS) {
				RCLCPP_ERROR(this->get_logger(), "Invalid axis number: %d. Must be between 0 and %d.", axis, MAX_MOTORS - 1);
				return;
			}
			
			{
				std::lock_guard<std::mutex> lock(queue_mutex_[axis]);
				goal_queues_[axis].push(goal_handle);
			}
			goal_condition_[axis].notify_one();  // Notify the execution thread
		}

		// This method creates an execution thread to process received goals sequentially
		void process_goals(int axis) {
			while (running_) {
				std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle;
				
				{
					std::unique_lock<std::mutex> lock(queue_mutex_[axis]);
					goal_condition_[axis].wait(lock, [this, axis] { return !goal_queues_[axis].empty() || !running_; });

					if (!running_) break;

					goal_handle = goal_queues_[axis].front();
					goal_queues_[axis].pop();
				}

				execute_goal(goal_handle, axis);
			}
		}


		// execution function for the requested goal position
		void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle, int axis) {
			auto result = std::make_shared<WireFeed::Result>();
			auto feedback = std::make_shared<WireFeed::Feedback>();
			const auto goal = goal_handle->get_goal();

			RCLCPP_INFO(this->get_logger(), "Starting wire feed for axis %d...", axis);

			// this assumes a single SC hub; iPort will need to be linked to each hub/node combo
			IPort &myPort = myMgr->Ports(0);
			INode &theNode = myPort.Nodes(axis);
			
			theNode.Motion.MoveWentDone();
			theNode.AccUnit(INode::RPM_PER_SEC);
			theNode.VelUnit(INode::RPM);
			theNode.Motion.AccLimit = goal->acceleration;
			theNode.Motion.VelLimit = goal->velocity;
			COUNTS_PER_REV = theNode.Info.PositioningResolution.Value();

			double distance_commanded = (goal->distance);
            theNode.Motion.AddToPosition(-theNode.Motion.PosnMeasured.Value());
			theNode.Motion.MovePosnStart(distance_commanded*COUNTS_PER_REV);
			
			double timeout = theNode.Motion.Adv.MovePosnHeadTailDurationMsec(goal->distance * COUNTS_PER_REV) + 100;
			double start_time = myMgr->TimeStampMsec();
			double distance_progress;

			while (!theNode.Motion.MoveWentDone() && myMgr->TimeStampMsec() < start_time + timeout) {
				if (!rclcpp::ok()) {
					RCLCPP_WARN(this->get_logger(), "ROS shutdown detected, aborting goal.");
					result->success = false;
					result->message = "Wire feed was NOT sucessful";
					goal_handle->abort(result);
					return;
				}

				// Provide periodic feedback
				distance_progress = theNode.Motion.PosnMeasured.Value()/COUNTS_PER_REV;
				feedback->current_distance = distance_progress;
				feedback->percent = distance_progress / distance_commanded * 100;
				feedback->torque_feedback = theNode.Motion.TrqMeasured.Value();


				goal_handle->publish_feedback(feedback);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Publish feedback every 100ms
			}

			RCLCPP_INFO(this->get_logger(), "Wire feed completed.");
			result->success = true;
			result->message = "Wire feed move was successful";
			goal_handle->succeed(result);
		}
	
		// Terminate the supervisor; only called on class destruction
		void quit_supervisor(){
			if (myMgr) {
				listOfNodes.clear();
		
				myMgr->PortsClose();

				myMgr = nullptr;

				RCLCPP_INFO(this->get_logger(), "Supervisor and all resources have been cleaned up.");
			}
		}
	
};

int main(int argc, char* argv[])
{
	std::cout.setf(std::ios::unitbuf);  // Disable buffering for std::cout
	setbuf(stdout, NULL);               // Disable buffering for printf

	// Fire up the ROS2 node and action server
	rclcpp::init(argc, argv);

	RCLCPP_INFO(rclcpp::get_logger("WireFeed"), "----------- RUNNING TEKNIC NODE -----------");

	{
		auto node = std::make_shared<WireFeedDistanceServer>();
		rclcpp::spin(node);
		node.reset();
	}

	rclcpp::shutdown();

	return 0;
}

