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
#include "Axis.h"
#include "Supervisor.h"

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
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

/************* Wire Feed Server Class Definition *********** */

using WireFeed = k2_action::action::WireFeed;

class WireFeedActionServer : public rclcpp::Node {
	public:
		// Constructor: build the action server and bind the major functions: handle_goal, handle_cancel, and handle_accepted
		WireFeedActionServer() : Node("wire_feed_action_server") {
			action_server_ = rclcpp_action::create_server<WireFeed>(
				this,
				"wire_feed_action",
				std::bind(&WireFeedActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
				std::bind(&WireFeedActionServer::handle_cancel, this, std::placeholders::_1),
				std::bind(&WireFeedActionServer::handle_accepted, this, std::placeholders::_1)
			);
			RCLCPP_INFO(this->get_logger(), "WireFeed Action Server is up and running!");

			// Setup the hardware
			initialize_axes();
		}
	private:
		std::vector<Axis*> listOfAxes;
		rclcpp_action::Server<WireFeed>::SharedPtr action_server_;
		float64 roller_diameter = 26.035; //mm - wire roller diameter
		size_t portCount = 0;

		//Create the SysManager object. This object will coordinate actions among various ports
		// and within nodes. In this example we use this object to setup and open our port.
		SysManager* myMgr = SysManager::Instance();
		std::vector<std::string> comHubPorts;

		// Create a pointer for the Supervisor member. This member formally gets initialized in initialize_axes()
		Supervisor* theSuper;

		// Assume that the nodes are of the right type and that this app has full control
		bool nodeTypesGood = true, accessLvlsGood = true; 

		// Method to initialize the SC Hubs as ports and motors as nodes
		void initialize_axes(){
			RCLCPP_INFO(this->get_logger(), "Initializing axes...");

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

					printf(" Port[%d]: state=%d, nodes=%d\n",
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

				for (unsigned iNode = 0; iNode < myPort.NodeCount(); iNode++){

					// Get a reference to the node, to make accessing it easier
					INode &theNode = myPort.Nodes(iNode);
					theNode.Motion.AccLimit = 100000;

					// Make sure we are talking to a ClearPath SC
					if (theNode.Info.NodeType() != IInfo::CLEARPATH_SC_ADV) {
						printf("---> ERROR: Uh-oh! Node %d is not a ClearPath-SC Advanced Motor\n", iNode);
						this->nodeTypesGood = false;
					}

					if (this->nodeTypesGood) {
						// Create an axis for this node
						this->listOfAxes.push_back(new Axis(&theNode));

						// Add the node to the trigger group
						theNode.Motion.Adv.TriggerGroup(1);
						
						if (!theNode.Setup.AccessLevelIsFull()) {
							printf("---> ERROR: Oh snap! Access level is not good for node %u\n", iNode);
							this->accessLvlsGood = false;
						}
					}

					// If we have full access to the nodes and they are all ClearPath-SC advanced nodes, 
					// then continue with the example
					if (nodeTypesGood && accessLvlsGood){

						// Create the supervisor thread, giving it access to the list of axes
						this->theSuper = new Supervisor(this->listOfAxes, *myMgr);
						theSuper->CreateThread();

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
		rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const WireFeed::Goal> goal) {
			RCLCPP_INFO(this->get_logger(), "Received goal request.");
			(void)uuid;
			(void)goal;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		
		// Cancel Callback
		rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
			RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
			(void)goal_handle;
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		// Goal Accepted Callback
		void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle) {
			RCLCPP_INFO(this->get_logger(), "Executing goal...");
			std::thread(&WireFeedActionServer::execute_goal, this, goal_handle).detach();
		}

		// execution function for the requested goal position
		void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WireFeed>> goal_handle){
			auto result = std::make_shared<WireFeed::Result>();
			RCLCPP_INFO(this->get_logger(), "Starting wire feed movement...");
			RCLCPP_INFO(this->get_logger(), "Supervisor State, current: %d, prev: %d", theSuper->m_state, theSuper->m_lastState);
			for (auto &axis : listOfAxes) {
				RCLCPP_INFO(this->get_logger(), "Axis State, current: %d, prev: %d", axis->m_state);
				axis->SetMoveRevs(3.0);
			}
			bool all_motors_done = false;
			while(!all_motors_done){
				all_motors_done = true;
				for (auto &axis : listOfAxes) {
					if (!axis->m_state==0) {
						// If the motor is not in idle yet, it's running still
						all_motors_done = false;
						//RCLCPP_INFO(this->get_logger(), "waiting for movement completion");
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			RCLCPP_INFO(this->get_logger(), "Wire feed movement completed.");
			goal_handle->succeed(result);
		}

		void quit_supervisor(){
			theSuper->Quit();
			theSuper->Terminate();
			delete theSuper;
			theSuper = nullptr;
						
			// Delete the list of axes that were created
			for (size_t iAxis = 0; iAxis < listOfAxes.size(); iAxis++){
				delete listOfAxes.at(iAxis);
			}
			listOfAxes.clear();

			// Close down the ports
			myMgr->PortsClose();

			RCLCPP_INFO(this->get_logger(), "Supervisor and all resources have been cleaned up.");
		}
};

int main(int argc, char* argv[])
{
	// Fire up the ROS2 node and action server
	rclcpp::init(argc, argv);
  	auto node = std::make_shared<WireFeedActionServer>();
	//msgUser("Multithreaded K2 instance starting. Press Enter to continue.");

	RCLCPP_INFO(rclcpp::get_logger("WireFeed"), "----------- RUNNING TEKNIC NODE -----------");

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

