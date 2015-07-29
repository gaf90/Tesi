#include "frontierauction.h"
#include "data/buddymessage.h"

namespace Coordination{
    FrontierAuction::FrontierAuction(QObject *parent)
    : QObject(parent), Auction<SLAM::Geometry::Frontier>(FRONTIER_AUCTION_TIMEOUT)
    {
        auctionInProgress = false;
        setAuctionType(FRONTIER);        
        connect(auctionTimer, SIGNAL(timeout()), this, SIGNAL(auctionExpired()));
    }

    FrontierAuction::~FrontierAuction()
    {
    }

    QHash<int, SLAM::Geometry::Frontier> FrontierAuction::assignItems()
    {
        Data::ObjectTable<int, uint, double> tempBids = bids;
        auctionInProgress = false;
        int tempRobotId;
        double bestEvaluation;
        int bestRobotId;
        uint bestFrontier;
        QHash<uint, double> tempEvaluations;
        uint tempFrontier;
        QHash<int, SLAM::Geometry::Frontier> assignedFrontiers;
        //TODO the following procedure is O(n^2*m), can be optimezed sorting first and then choosing
        while((tempBids.rowCount() > 0) && (tempBids.columnCount() > 0)){
            //exists at least a robot without an assigned frontier and there is at least a frontier that can be assigned            
            bestRobotId = tempBids.keys().first();
            bestFrontier = tempBids[bestRobotId].keys().first();
            bestEvaluation = tempBids.getValue(bestRobotId, bestFrontier);


            foreach(tempRobotId, tempBids.keys()){
                tempEvaluations = tempBids[tempRobotId];
                foreach(tempFrontier, tempEvaluations.keys()){
                    if(tempEvaluations[tempFrontier] > bestEvaluation){
                        bestEvaluation = tempEvaluations[tempFrontier];
                        bestRobotId = tempRobotId;
                        bestFrontier = tempFrontier;
                    }
                }
            }
            assignedFrontiers.insert(bestRobotId, items[bestFrontier]);           

            //remove from tempBids the assigned robot and the assigned frontier
            tempBids.deleteRow(bestRobotId);
            tempBids.deleteColumn(bestFrontier);
        }        
        return assignedFrontiers;
//        double tempValue;
//        QHash<int, SLAM::Geometry::Frontier> finalAssignedFrontiers;
//        foreach(int robotId, assignedFrontiers.keys()){
//            tempValue = bids.getValue(robotId, qHash(assignedFrontiers[robotId]));
//            if(tempValue != ITEM_NOT_INTERESTING)
//                finalAssignedFrontiers.insert(robotId, assignedFrontiers[robotId]);
//        }
//        return finalAssignedFrontiers;
    }

    void FrontierAuction::onAuctionExpired()
    {
        emit auctionExpired();
    }
}
