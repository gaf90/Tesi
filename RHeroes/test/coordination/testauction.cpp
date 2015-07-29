#include "testauction.h"
#include "coordination/auction/auction.h"

namespace Test{
    TestAuction::TestAuction(QObject *parent) :
        QObject(parent)
    {
    }

    TestAuction::~TestAuction()
    {
    }

    void TestAuction::testAddItems()
    {
        /*Coordination::Auction<int> auction = Coordination::Auction<int>();
        QList<int> lstInt;
        int item = 66;
        lstInt << 1 << 2 << 5 << 17;
        auction.addItem(&item);
        auction.addItems(&lstInt);
        QVERIFY2(auction.getItems()->count() == 0, "MLADENNNNNNNN");
        auction.startNewAuction(666);
        auction.addItem(&item);
        QVERIFY2(auction.getItems()->count() == 1, "MLADENNNNNNNN");
        auction.addItems(&lstInt);
        QVERIFY2(auction.getItems()->count() == 5, "MLADENNNNNNNN");
        */
    }

    void TestAuction::testAddBids()
    {
        /*Coordination::Auction<int> auction = Coordination::Auction<int>();
        QList<int> lstInt;
        lstInt << 1 << 2 << 5 << 17;
        auction.startNewAuction(25);
        auction.addItems(&lstInt);
        QHash<uint, double> bids;
        int tempInt;
        foreach(tempInt, lstInt)
            bids.insert(qHash(tempInt), tempInt);
        bids.remove(qHash(lstInt.first()));
        QVERIFY(!auction.addBids(1, bids)); //it is not possible to add a bids on a subset of the items
        bids.insert(qHash(33), 33);
        QVERIFY(!auction.addBids(1, bids)); //a bid must involve all and only all the items
        bids.remove(qHash(33));
        bids.insert(qHash(lstInt.first()), lstInt.first());
        QVERIFY(auction.addBids(1, bids)); //right bids
        QVERIFY(!auction.addBids(1, bids)); //robot has alreay bidded
        auction.endAuction();*/
    }
}
