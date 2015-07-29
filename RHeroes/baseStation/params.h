#ifndef PARAMS_H
#define PARAMS_H

/**
* This file is used to store every constant value used in the base station process.
* @author Alain Caltieri
*/
namespace BaseStation{

/* ========================================================================== */
/*                            MESSAGE MANAGER                                 */
/* ========================================================================== */

//! The number of colums in the QTableView that shows messages
#define MESSAGE_VIEWER_COLUMS 7

//! Types of messages allowed and initial values of relative params
#define POSSIBLE_MESSAGES "victim-100-50-80,fault-80-100-100,semantic-10-10-0,feedback-50-100-30"
#define VICTIM_MEX_PARAMS "0.5,0.5,0.5"
#define FAULT_MEX_PARAMS "0.5,0.5,0.5"
#define SEMATIC_MEX_PARAMS "0.5,0.5,0.5"
//! keywords for messages' types! These must match with the POSSIBLE_MESSAGES keys!!!
#define VICTIM_TYPE "victim"
#define FAULT_TYPE "fault"
#define SEMANTIC_TYPE "semantic"
#define FEEDBACK_TYPE "feedback"

//! headers
#define NOTIFICATION_HEADERS "id/Robot id/Priority/Module/Message"
#define ARCHIVIATION_HEADER "Useful"
#define DELETION_HEADER "Useless"
#define MODULE_COLUMN 3
#define PRIORITY_COLUMN 2
#define NN_ROBOT_COLUMN 1

//! Params for the message priority calculator
#define NM_TIME_THRESOLD 10
#define NM_MAXIMAL_PRIORITY 100
#define NM_AFFIDABILITY_WEIGHT 0.3 * NM_MAXIMAL_PRIORITY
#define NM_RELEVANCE_WEIGHT 0.5 * NM_MAXIMAL_PRIORITY
#define NM_USER_INTEREST_WEIGHT 0.2 * NM_MAXIMAL_PRIORITY
#define NM_WORKLOAD_WEIGHT 0.05 * NM_MAXIMAL_PRIORITY

//------------------------ User Interest params ------------------------------//
//! time for user interaction counter reduction, measured in seconds.
#define NM_TIME_FOR_INTERACTION_REDUCTION 5
#define NM_MAX_USER_INTEREST_FACTOR 10

//------------------------- Reliability params -------------------------------//
#define NM_MAXIMAL_RELIABILITY 100.
#define NM_SINGLE_MESSAGE_EFFECT 5

/* ========================================================================== */
/*                             VICTIM MANAGER                                 */
/* ========================================================================== */

#define VICTIM_VIEWER_COLUMNS 6
#define VICTIM_HEADERS "Victim id,Position,Assigned Robot,Discoverer,Time to reach"
#define VICTIM_DELETION_HEADER "Remove"
#define VICTIM_ASSOCIATION_COLUMN 2
#define TIME_TO_REACH_VICTIM_COLUMN 4
#define NO_VICTIM 999

}
#endif // PARAMS_H
