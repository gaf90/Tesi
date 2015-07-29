#include "victimauction.h"
#include "coordination/coordinationconstants.h"
#define ROBOT_ESTIMATE_WORSENING_FACTOR 50


namespace Coordination{
    VictimAuction::VictimAuction(QObject *parent)
        : QObject(parent), Auction<uint>(VICTIM_AUCTION_TIMEOUT)
    {
        auctionInProgress = false;
        setAuctionType(VICTIM);        
        connect(auctionTimer, SIGNAL(timeout()), this, SIGNAL(auctionExpired()));
    }

    VictimAuction::~VictimAuction()
    {
    }

    QHash<int, uint> VictimAuction::assignItems()
    {
        Data::ObjectTable<int, uint, double> tempBids = bids;
        QList<int> alreadyBusyRobot;
        auctionInProgress = false;
        int bestRobotId;
        double bestEvaluation;
        QHash<int, uint> assignedVictims;
        int tempRobotId;
        uint bestVictim;
        QHash<uint, double> tempEvaluations;
        uint tempVictim;

        while(tempBids.columnCount() > 0/* && tempBids.rowCount() > 0*/){ //if there are no robots it is useless (may happen when auctioneer has alrready finalVictimAssigned and no one else joins the auction
            //exists at least a victim not yet assigned
            bestRobotId = tempBids.keys().first();
            bestVictim = tempBids[bestRobotId].keys().first();
            bestEvaluation = tempBids.getValue(bestRobotId, bestVictim);

            foreach(tempRobotId, tempBids.keys()){
                if(alreadyBusyRobot.contains(tempRobotId))
                    continue; //robot has already a victim (or more), can't have another one if exists a robot with less victims
                //ldbg << "evaluations of robot" << tempRobotId << ": ";
                tempEvaluations = tempBids[tempRobotId];
                foreach(tempVictim, tempEvaluations.keys()){
                    //ldbg << " (victim" << tempVictim << ", " << tempEvaluations[tempVictim] << ")";
                    if(tempEvaluations[tempVictim] < bestEvaluation){
                        bestEvaluation = tempEvaluations[tempVictim];
                        bestRobotId = tempRobotId;
                        bestVictim = tempVictim;
                    }
                    //ldbg << endl;
                }
            }
            assignedVictims.insertMulti(bestRobotId, items[bestVictim]);
            alreadyBusyRobot.append(bestRobotId);

            //ldbg << "victims not yet assigned(" << tempBids.columnCount() << "):";
            foreach(tempVictim, tempEvaluations.keys()){
                //ldbg << " " << tempVictim;
            }
            //ldbg << endl;


            //remove from tempBids the assigned victim
            tempBids.deleteColumn(bestVictim);
            //multiply
            tempBids.multiplyForFactorRow(bestRobotId, ROBOT_ESTIMATE_WORSENING_FACTOR);
            //ldbg << "assigned victim" << bestVictim << " to robot" << bestRobotId << endl;
            if((assignedVictims.size() % alreadyBusyRobot.size()) == 0){
                alreadyBusyRobot.clear(); //all robots already have a victim, they will have a second (and more) one
            }
        }        
        return assignedVictims;
    }

    void VictimAuction::onAuctionExpired()
    {
        emit auctionExpired();
    }
}
