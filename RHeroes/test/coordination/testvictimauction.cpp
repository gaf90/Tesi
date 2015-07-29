#include "testvictimauction.h"

namespace Test{
TestVictimAuction::TestVictimAuction(QObject *parent) :
    QObject(parent)
{    
}

void TestVictimAuction::testVictimAllocation()
{
    /* FIRST SET
     *      v0   v1   v2
     * r0   10   8     5
     * r1   13   2     4
     * r2    1   6     2
     */
    using namespace Coordination;
    VictimAuction victimAuction;
    int victimCount;
    int robotCount;
    QList<double> bid1, bid2, bid3;
    bid1 << 10 << 8 << 5;
    bid2 << 13 << 2 << 4;
    bid3 << 1 << 6 << 1;
    QList<QList<double> > bidsList;
    bidsList << bid1 << bid2 << bid3;
    QHash<int, uint> assignments;
    QHash<uint, int> invertedAssignments; //needed for qverify

    //FIRST AUCTION
    victimAuction.startNewAuction(); //final assignments: (r2,v0) (r1,v1) (r0,v2)
    victimCount = 3;
    robotCount = 3;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    QHash<int, uint>::const_iterator i;
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 2);
    QVERIFY(invertedAssignments[1] == 1);
    QVERIFY(invertedAssignments[2] == 0);
    victimAuction.endAuction();


    //SECOND AUCTION
    victimAuction.startNewAuction(); //final assignments: (r0,v0) (r1,v1) (r0,v2)
    victimCount = 3;
    robotCount = 2;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 0);
    QVERIFY(invertedAssignments[1] == 1);
    QVERIFY(invertedAssignments[2] == 0);
    victimAuction.endAuction();

    //THIRD AUCTION
    victimAuction.startNewAuction(); //final assignments: (r2,v0) (r1,v1)
    victimCount = 2;
    robotCount = 3;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 2);
    QVERIFY(invertedAssignments[1] == 1);
    victimAuction.endAuction();



    /* SECOND SET
     *      v0   v1   v2    v3   v4   v5
     * r0   10   8     5     6    9   16
     * r1   13   2     4    21   23    1
     * r2    1   6     2     2    5   98
     * r3    3  81     3    16    9   16
     * r4    4  12    22    27   23   11
     */

    QList<double> bid4, bid5, bid6;
    bid1.clear();
    bid2.clear();
    bid3.clear();
    bid1 << 10 << 8 << 5 << 6 << 9 << 16;
    bid2 << 13 << 2 << 4 << 21 << 23 << 1;
    bid3 << 1 << 6 << 2 << 2 << 5 << 98;
    bid4 << 3 << 81 << 3 << 16 << 9 << 16;
    bid5 << 4 << 12 << 22 << 27 << 23 << 11;
    bid6 << 6 << 14 << 212 << 21 << 15 << 91;
    bidsList.clear();
    bidsList << bid1 << bid2 << bid3 << bid4 << bid5 << bid6;

    // FIRST AUCTION
    victimAuction.startNewAuction(); //final assignments: (r1,v5) (r2,v0) (r3,v2) (r0,v3) (r4,v1) (r2,v4)
    victimCount = 6;
    robotCount = 5;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 2);
    QVERIFY(invertedAssignments[1] == 4);
    QVERIFY(invertedAssignments[2] == 3);
    QVERIFY(invertedAssignments[3] == 0);
    QVERIFY(invertedAssignments[4] == 2);
    QVERIFY(invertedAssignments[5] == 1);
    victimAuction.endAuction();

    // SECOND AUCTION
    victimAuction.startNewAuction(); //final assignments: (r1,v5) (r2,v0) (r0,v2) (r1,v1) (r2,v3) (r0,v4)
    victimCount = 6;
    robotCount = 3;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 2);
    QVERIFY(invertedAssignments[1] == 1);
    QVERIFY(invertedAssignments[2] == 0);
    QVERIFY(invertedAssignments[3] == 2);
    QVERIFY(invertedAssignments[4] == 0);
    QVERIFY(invertedAssignments[5] == 1);
    victimAuction.endAuction();

    // THIRD AUCTION
    victimAuction.startNewAuction(); //final assignments: (r2,v0) (r1,v1) (r3,v2) (r0,v3)
    victimCount = 4;
    robotCount = 5;
    addBids(victimAuction, bidsList, robotCount, victimCount);
    assignments = victimAuction.assignItems();
    ldbg << "assignment size: " << assignments.size() << ", victim: " << victimCount << endl;
    QVERIFY(assignments.size() == victimCount);
    invertedAssignments.clear();
    for (i = assignments.constBegin(); i != assignments.constEnd(); ++i){
        ldbg << "robot" << i.key() << " coupled with victim" << i.value() << endl;
        invertedAssignments.insert(i.value(), i.key());
    }
    QVERIFY(invertedAssignments.size() == victimCount);
    QVERIFY(invertedAssignments[0] == 2);
    QVERIFY(invertedAssignments[1] == 1);
    QVERIFY(invertedAssignments[2] == 3);
    QVERIFY(invertedAssignments[3] == 0);
    victimAuction.endAuction();

}

void TestVictimAuction::addBids(Coordination::VictimAuction &victimAuction, QList<QList<double> > &bidsList, int robotCount, int victimCount)
{
    QList<uint> victimLst;
    QHash<uint, double> bids;
    for(int i = 0; i < victimCount; i++)
        victimLst.append(i);
    victimAuction.addItems(&victimLst);

    for(int i=0; i < robotCount; i++){
        bids.clear();
        for(int k=0; k < victimCount; k++){
            bids.insert(k, bidsList.at(i).at(k));
        }
        victimAuction.addBids(i, bids);
    }
}

}
