#ifndef COORDINATIONCOSTANTS_H
#define COORDINATIONCOSTANTS_H

//coordination message module
#define CHECK_MESSAGE_TIMEOUT 100

//frontier

#define FRONTIER_AUCTION_TIMEOUT 500
#define ITEM_NOT_INTERESTING 0 //represents a bid on an item not wanted (necessary if among the items there is at least one interesting),
#define MINIMUM_FRONTIER_DISTANCE_FROM_PREVIOUS 1 //minimum distance to consider a frontier a new frontier, calculated with respect to centroids
#define MINIMUM_EVALUATION_INCREASE 0.3141592653589793238 //minimum increase of evaluation from previous frontier to accept the new one assigned
#define EVALUATION_HISTORY_DELETE_PERCENTAGE 0.2 // percentage of evaluations deleted after frontier assignment

//victim
#define VICTIM_AUCTION_TIMEOUT 2000
#define VICTIM_RESEND_FOR_ACK_INTERVAL 10000
#define CHECK_AUCTION_REQUIRED_INTERVAL 25000
#define CHECK_GO_CLOSE_TO_A_VICTIM_REQUIRED_INTERVAL 5000
#define GO_TO_VICTIM_THRESHOLD 1.2 //20% more time than required
#define TIME_TO_GO_TO_VICTIM_THRESHOLD 600000 //if remaining time less than this value, then force to go close to a victim
#endif // COORDINATIONCOSTANTS_H
