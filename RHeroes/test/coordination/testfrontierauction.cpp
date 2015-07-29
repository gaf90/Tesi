#include "testfrontierauction.h"
#include "slam/geometry/frontier.h"
#include "coordination/auction/frontierauction.h"

namespace Test{
    TestFrontierAuction::TestFrontierAuction(QObject *parent) :
        QObject(parent)
    {
    }

    TestFrontierAuction::~TestFrontierAuction()
    {
    }

    void TestFrontierAuction::testMessages()
    {
        /*
        //create auction:
        int auctioneerId = 1;
        int bidderId = 2;
        int auctionId = 33;        
        Coordination::FrontierAuction frontierAuction = Coordination::FrontierAuction();
        frontierAuction.startNewAuction(auctionId);
        QVERIFY(frontierAuction.getItems()->count() == 0);
        using namespace SLAM::Geometry;
        Frontier tempFrontier;
        Frontier fr1 = Frontier(0,0,1,1);
        Frontier fr2 = Frontier(0,0,11,111);
        Frontier fr3 = Frontier(10,10,1,1);
        QList<Frontier> lstFrontier;
        lstFrontier << fr1 << fr2 << fr3;
        frontierAuction.addItems(&lstFrontier);
        QVERIFY(frontierAuction.getItems()->count() == lstFrontier.size());

        //create send parse AskBidsMessage
        Data::WirelessMessage askBidsMessageSent = frontierAuction.createAskBidsMessage(auctioneerId);       

        /*const Data::AskBidsMessage<Frontier> * askBidsMessageReceived = frontierAuction.parseAskBidsMessage(askBidsMessageSent);

        QVERIFY(askBidsMessageReceived->getAuctionId() == auctionId);
        QVERIFY(askBidsMessageReceived->getSenderId() == auctioneerId);
        QVERIFY(askBidsMessageReceived->getItems()->count() == lstFrontier.size());
        QList<Frontier> lstFrontierReceived = * askBidsMessageReceived->getItems();
        foreach(tempFrontier, lstFrontierReceived){
            QVERIFY(lstFrontier.contains(tempFrontier));
        }
        foreach(tempFrontier, lstFrontier){
            QVERIFY(lstFrontierReceived.contains(tempFrontier));
        }

        //create send parse SendBidsMessage
        QHash<uint, double> bids;
        QList<double> bidsValue;
        bidsValue << 1 << 5 << 10 << 145 << 25 << 30;
        for(int i=0; i<lstFrontier.size(); i++)
            bids.insert(qHash(tempFrontier), bidsValue.at(i));
        Data::WirelessMessage sendBidsMessageSent = frontierAuction.createSendBidsMessage(auctionId, bidderId, bids, auctioneerId);

        const Data::SendBidsMessage * sendBidsMessageReceived = frontierAuction.parseSendBidsMessage(sendBidsMessageSent);
        QVERIFY(sendBidsMessageReceived->getAuctionId() == auctionId);
        QVERIFY(sendBidsMessageReceived->getBidderId() == bidderId);
        QVERIFY(*sendBidsMessageReceived->getBids() == bids);

        //create send parse AssignItemMessage
        Data::WirelessMessage assignItemMessageSent = frontierAuction.createAssignItemMessage(&lstFrontier.first(), auctioneerId, bidderId);

        const Data::AssignItemMessage<Frontier> * assignItemMessageReceived = frontierAuction.parseAssignItemMessage(assignItemMessageSent);
        QVERIFY(*assignItemMessageReceived->getItem() == lstFrontier.first());
        */
    }


    void TestFrontierAuction::testAssignFrontiers()
    {
        /*int auctionId = 33;
        Coordination::FrontierAuction frontierAuction = Coordination::FrontierAuction();
        using namespace SLAM::Geometry;
        /*
         *          fr1   fr2   fr3
         *  robot0   1     5     4
         *  robot1  10     3     2
         *  robot2   3     9    11
         *  Result  3 robots | 3 frontiers: (robot2, fr3) ==> (robot1, fr1) ==> (robot0, fr2)
         *  Result 3 robots | 2 frontiers: (robot1, fr1) ==> (robot2, fr2)
         *  Result 2 robots | 3 frontiers: (robot1, fr1) ==> (robot0, fr2)
        */

        /*
        QList<Frontier> lstFrontiers;
        Frontier fr1(1,1, 3,3);
        Frontier fr2(2,1, 3,5);
        Frontier fr3(3,1, 2,3);
        lstFrontiers << fr1 << fr2 << fr3;

        QList<double> lstBids1, lstBids2, lstBids3;
        lstBids1 << 1 << 5 << 4;
        lstBids2 << 10 << 3 << 2;
        lstBids3 << 3 << 9 << 11;

        QList<QList<double> > lstBids;
        lstBids << lstBids1 << lstBids2 << lstBids3;

        int nRobots;
        int nFrontiers;
        QHash<uint, double> bids;
        QHash<int, Frontier> assignment;

        //first auction: 3 robots and 3 frontiers
        nRobots = 3;
        nFrontiers = 3;
        frontierAuction.startNewAuction(auctionId);
        frontierAuction.addItems(&lstFrontiers);
        for(int i = 0; i < nRobots; i++){
            bids.clear();
            for(int k = 0; k < nFrontiers; k++){
                bids.insert(qHash(lstFrontiers.at(k)), lstBids.at(i).at(k));
            }
            frontierAuction.addBids(i, bids);
        }
        assignment = frontierAuction.assignItems();
        QVERIFY(assignment[0] == fr2);
        QVERIFY(assignment[1] == fr1);
        QVERIFY(assignment[2] == fr3);
        frontierAuction.endAuction();


        //second auction: 3 robots and 2 frontiers
        nRobots = 3;
        nFrontiers = 2;
        frontierAuction.startNewAuction(auctionId);
        for(int i = 0; i < nFrontiers; i++)
            frontierAuction.addItem(&lstFrontiers[i]);

        for(int i = 0; i < nRobots; i++){
            bids.clear();
            for(int k = 0; k < nFrontiers; k++){
                bids.insert(qHash(lstFrontiers.at(k)), lstBids.at(i).at(k));
            }
            frontierAuction.addBids(i, bids);
        }
        assignment = frontierAuction.assignItems();
        QVERIFY(assignment[1] == fr1);
        QVERIFY(assignment[2] == fr2);
        QVERIFY(!assignment.contains(0));
        frontierAuction.endAuction();


        //third auction: 2 robots and 3 frontiers
        nRobots = 2;
        nFrontiers = 3;
        frontierAuction.startNewAuction(auctionId);
        for(int i = 0; i < nFrontiers; i++)
            frontierAuction.addItem(&lstFrontiers[i]);

        for(int i = 0; i < nRobots; i++){
            bids.clear();
            for(int k = 0; k < nFrontiers; k++){
                bids.insert(qHash(lstFrontiers.at(k)), lstBids.at(i).at(k));
            }
            frontierAuction.addBids(i, bids);
        }
        assignment = frontierAuction.assignItems();
        QVERIFY(assignment[0] == fr2);
        QVERIFY(assignment[1] == fr1);
        QVERIFY(!assignment.contains(2));
        frontierAuction.endAuction();
        */
    }
}
