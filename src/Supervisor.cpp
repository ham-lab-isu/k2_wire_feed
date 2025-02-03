///*******************************************************************************
//
// NAME
//		Supervisor
//
// DESCRIPTION:
//		This machine initializes all the nodes and runs the state machine
//		that synchronizes the moves among the nodes; the synchronization
//		is simply waiting for all nodes to send their moves to their nodes,
//		triggering them all at the same time, then waiting for them all to
//		finish before issuing any more moves
//
//																			   *
//******************************************************************************

//******************************************************************************
// NAME																		   *
// 	Supervisor.cpp headers
//
#include "Supervisor.h"
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::Supervisor
//
//	DESCRIPTION:
//		Constructor for the supervisor state
//
Supervisor::Supervisor(vector<Axis*> theAxes, SysManager &myMgr) 
	: m_sysMgr(myMgr)
{
	m_axes = theAxes;
	m_state = SUPER_IDLE;
	m_quitting = false;
	// create list of flags to keep track of the nodes for
	// various conditions
	for (Uint16 iAxis = 0; iAxis < m_axes.size(); iAxis++){
		m_axisIdle.push_back(false);
		m_axisReady.push_back(false);
		m_axisSentMove.push_back(false);
		m_axisDone.push_back(false);
	}
}

void Supervisor::CreateThread(){
	printf("[Supervisor] Creating Supervisor thread...\n");
	m_thread = thread(&Supervisor::SuperMain, this);
	m_thread.detach();
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::SignaleIdle
//
//	DESCRIPTION:
//		Nodes signal to the supervisor when they are idle
//
void Supervisor::SignalIdle(Uint16 nodeNum){
	SignalPerNode(m_axisIdle, m_nodesIdle, nodeNum);
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::SignalReady
//
//	DESCRIPTION:
//		Nodes signal to the supervisor when they are ready to go
//
void Supervisor::SignalReady(Uint16 nodeNum){
	SignalPerNode(m_axisReady, m_nodesReady, nodeNum);
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::WaitForKick
//
//	DESCRIPTION:
//		Nodes will wait to be kicked
//
void Supervisor::WaitForKick(){
	WaitForCondition(m_nodesKicked);
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::SignalMoveSent
//
//	DESCRIPTION:
//		Nodes signal that they have sent their moves
//
void Supervisor::SignalMoveSent(Uint16 nodeNum){
	SignalPerNode(m_axisSentMove, m_nodesMoved, nodeNum);
}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::SignalMoveDone
//
//	DESCRIPTION:
//		Nodes signal to the supervisor when their move is done
//
void Supervisor::SignalMoveDone(Uint16 nodeNum){
	SignalPerNode(m_axisDone, m_nodesDone, nodeNum);
	printf("[Supervisor::SignalMoveDone] Move has been signalled as done by the Axis.\n");

    // Only proceed if ALL nodes are done
    bool all_done = true;
    for (bool done : m_axisDone) {
        if (!done) {
            all_done = false;
            break;
        }
    }

    if (all_done) {
        printf("DEBUG: All axes have completed their moves. Notifying Supervisor.\n");
        SetCondition(m_nodesDone);
    }}
//																			   *
//******************************************************************************

//******************************************************************************
//	NAME																	   *
//		Supervisor::RequestMove and Supervisor::IsMoveRequested
//
//	DESCRIPTION:
//		Method to explicitly call move requests
//
void Supervisor::RequestMove() {
    std::lock_guard<std::mutex> lock(m_mutex);  // Ensure thread safety
    m_moveRequested = true;
    printf("[Supervisor] Move has been explicitly requested.\n");
}

bool Supervisor::IsMoveRequested() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_moveRequested;
}


//******************************************************************************
//	NAME																	   *
//		Supervisor::SuperMain
//
//	DESCRIPTION:
//		The Supervisor's state machine thread; tells each Axis to create its
//		own thread and initialize itself, then runs the state machine to
//		tell the Axis objects when to send their moves, triggers the moves
//		after they've all been sent, and waits for all the nodes to finish 
//		their move before starting the process again.
//
void Supervisor::SuperMain(){
	// Get all conditions to a known state
	ResetCondition(m_axisIdle, m_nodesIdle);
	ResetCondition(m_axisSentMove, m_nodesMoved);
	ResetCondition(m_axisDone, m_nodesDone);
	ResetCondition(m_nodesKicked);
	ResetCondition(m_nodesReady);
	try{
		// Create threads for all the axes
		for (Uint16 iAxis = 0; iAxis < m_axes.size(); iAxis++){
			printf(" axis[%d]: userID=%s\n", iAxis, m_axes.at(iAxis)->MyNode()->Info.UserID.Value());
			m_axes.at(iAxis)->CreateThread(this);
		}
	}
	catch(...){
		printf("Supervisor failed initializing threads\n");
		Quit();
	}

	// Get the state machine going
	while (!m_quitting){
		try {
			// For debugging
			printf("Supervisor State is currently %d and was previously %d\n", m_state, m_lastState);
			if (m_nodesDone){
				printf("Supervisor is currently seeing m_nodesDone as TRUE\n");
			}

			m_lastState = m_state;

			switch (m_state){
				case SUPER_IDLE:
					// Reset each node to idle
					for (Uint16 iAxis = 0; iAxis < m_axes.size(); iAxis++)
						m_axes.at(iAxis)->ResetToIdle();
					m_state = SUPER_WAIT_FOR_IDLE;
					break;
				case SUPER_WAIT_FOR_IDLE:
					if (m_quitting)
						continue;

					WaitForCondition(m_nodesIdle);
					
					if (m_quitting)
						continue;

					ResetCondition(m_axisIdle, m_nodesIdle);

					// Only kick nodes if a move has been requested
					if (IsMoveRequested()) {
						printf("[Supervisor] Kicking nodes for a new move.\n");
						SetCondition(m_nodesKicked);
						m_moveRequested = false;  // Reset the move request flag
						m_state = SUPER_WAIT_FOR_MOVES_SENT;
					} else {
						printf("[Supervisor] No move requested, staying in idle state.\n");
						m_state = SUPER_WAIT_FOR_IDLE;
					}
					break;
				case SUPER_WAIT_FOR_MOVES_SENT:
					// Wait for them all to say their moves have been sent
					WaitForCondition(m_nodesMoved);
					if (m_quitting)
						continue;
					ResetCondition(m_nodesKicked);
					ResetCondition(m_axisSentMove, m_nodesMoved);
					printf("Triggering moves in group\n");
					m_axes.at(0)->MyNode()->Port.Adv.TriggerMovesInGroup(1);
					m_state = SUPER_WAIT_FOR_MOVES_DONE;
					printf("Moves have been triggered and state set to waiting for moves to be done.\n");
					break;
				case SUPER_WAIT_FOR_MOVES_DONE:
					WaitForCondition(m_nodesDone);
					printf("DEBUG: Move completion detected by Supervisor, resetting conditions.\n");
					PrintStats();
					// Take a break
					m_sysMgr.Delay(10);
					if (m_quitting)
						continue;
					ResetCondition(m_axisDone, m_nodesDone);
					m_state = SUPER_WAIT_FOR_IDLE;
					break;
				default:
					printf("Super: Undefined state %d\n", m_state);
					return;
			}
		} // Try block
		catch (mnErr &err) {
			fprintf(stderr, "Supervisor thread failed: %s\n", err.ErrorMsg);
			Quit();
		}
		catch(...) {
			fprintf(stderr, "Supervisor thread failed.\n");
			Quit();
		}
		//m_sysMgr.Delay(0);
	}

	SetCondition(m_nodesKicked);
	WaitForCondition(m_nodesIdle);

	m_state = SUPER_EXITED;
	// Wait for threads to end
	for (Uint16 iAxis = 0; iAxis < m_axes.size(); iAxis++){
		m_axes.at(iAxis)->Join();
	}
	return;
}
//																			   *
//******************************************************************************
