#include "testrrtalgorithm.h"
#include "pathPlanner/RRT/rrtalgorithm.h"
#include "shared/utilities.h"
#include <QStack>
#include "data/action.h"
#include "pathPlanner/pathplannerutils.h"

namespace Test {

using namespace PathPlanner;
using namespace Data;

TestRRTAlgorithm::TestRRTAlgorithm(QObject *parent) :
    QObject(parent)
{

}

TestRRTAlgorithm::~TestRRTAlgorithm()
{

}

void TestRRTAlgorithm::testComputeLowLevelAction()
{
    RRTNode start(0.0, 0.0), goal(1.0, 1.0);
    SLAM::Map map;
    RRTAlgorithm algorithm(&map, &start, &goal);

    goal.setParent(&start); start.addReachable(&goal);
    bool valid = computeOrientation(&goal, &start, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QVERIFY2(goal.getTheta() == fromDegreeToRadiants(45), "When I reach the goal i should be at 45°");

    QStack<Action *> *todo = algorithm.computeLowLevelAction();
    Action * act = todo->pop();
    QVERIFY(act->getType() == Action::Rotation);
    QVERIFY(act->getValue() == fromDegreeToRadiants(45));
    delete act;

    act = todo->pop();
    QVERIFY(act->getType() == Action::Translation);
    QVERIFY(act->getValue() == EuclideanDistance(start, goal));
    delete act;


}

void TestRRTAlgorithm::testRandomNode()
{

}

void TestRRTAlgorithm::testNearNode()
{
    RRTNode start(0.0, 0.0), goal(15.0, 15.0);
    SLAM::Map map;
    RRTAlgorithm algorithm(&map, &start, &goal);
    RRTNode * ret = algorithm.nearNode(&start, &goal, 10);
    QVERIFY(EuclideanDistance(start, *ret) == 10);
    delete ret;

    goal.setX(9.0);
    goal.setY(0.0);
    ret = algorithm.nearNode(&start, &goal, 10);
    QVERIFY(ret == &goal);


}

void TestRRTAlgorithm::testClosestNode()
{
    RRTNode start(0.0, 0.0), goal(1.0, 1.0);
    SLAM::Map map;
    RRTAlgorithm algorithm(&map, &start, &goal);
    RRTNode * ret = algorithm.closestNode(&start, &goal);
    QVERIFY(&start == ret);

    RRTNode *mid = new RRTNode(0.5, 0.5);
    start.addReachable(mid); mid->setParent(&start);
    algorithm.generated->insert(mid);
    ret = algorithm.closestNode(&start, &goal);
    QCOMPARE(mid, ret);

    RRTNode rand(-1.0, -1.0);
    ret = algorithm.closestNode(&start, &rand);
    QVERIFY(&start == ret);

}

void TestRRTAlgorithm::testConstrainedClosestNode()
{

}

void TestRRTAlgorithm::testUnconstrainedComputeOrientation()
{
    RRTNode start(0.0, 0.0), goal(1.0, 1.0);
    start.setParent(NULL); goal.setParent(NULL);
    start.setSteeringAngle(0.0); goal.setSteeringAngle(0.0);
    start.setTheta(0.0); goal.setTheta(0.0);
    start.setValid(true); goal.setValid(true);

    SLAM::Map map;
    RRTAlgorithm algorithm(&map, &start, &goal);
    bool valid = computeOrientation(&goal, &start, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QVERIFY2(goal.getTheta() == fromDegreeToRadiants(45), "When I reach the goal i should be at 45°");

    start.setTheta(fromDegreeToRadiants(-90));
    valid = computeOrientation(&goal, &start, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(45+90), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QVERIFY2(goal.getTheta() == fromDegreeToRadiants(45), "When I reach the goal i should be at 45°");

    start.setTheta(fromDegreeToRadiants(90));
    valid = computeOrientation(&goal, &start, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(-45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(goal.getTheta(),  fromDegreeToRadiants(45));

    start.setTheta(fromDegreeToRadiants(180));
    valid = computeOrientation(&goal, &start, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(-135), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(goal.getTheta(),  fromDegreeToRadiants(45));

    start.setTheta(0.0); goal.setTheta(0.0);
    valid = computeOrientation(&start, &goal, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(-135), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(90));
    valid = computeOrientation(&start, &goal, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QCOMPARE(start.getSteeringAngle(), fromDegreeToRadiants(135));
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(-90));
    valid = computeOrientation(&start, &goal, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(-45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(180));
    valid = computeOrientation(&start, &goal, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(-180));
    valid = computeOrientation(&start, &goal, -1);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));
}

void TestRRTAlgorithm::testConstrainedComputeOrientation()
{
    RRTNode start(0.0, 0.0), goal(1.0, 1.0);
    start.setParent(NULL); goal.setParent(NULL);
    start.setSteeringAngle(0.0); goal.setSteeringAngle(0.0);
    start.setTheta(0.0); goal.setTheta(0.0);
    start.setValid(true); goal.setValid(true);

    SLAM::Map map;
    RRTAlgorithm algorithm(&map, &start, &goal, 60);
    bool valid = computeOrientation(&goal, &start, 60);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QVERIFY2(goal.getTheta() == fromDegreeToRadiants(45), "When I reach the goal i should be at 45°");

    start.setTheta(fromDegreeToRadiants(-90));
    valid = computeOrientation(&goal, &start, 60);
    QVERIFY2(!valid, "no constraints. Should be True");

    start.setTheta(fromDegreeToRadiants(90));
    valid = computeOrientation(&goal, &start, 60);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(goal.getSteeringAngle() == fromDegreeToRadiants(-45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(goal.getTheta(),  fromDegreeToRadiants(45));

    start.setTheta(fromDegreeToRadiants(180));
    valid = computeOrientation(&goal, &start, 60);
    QVERIFY2(!valid, "no constraints. Should be True");

    start.setTheta(0.0); goal.setTheta(0.0);
    valid = computeOrientation(&start, &goal, 60);
    QVERIFY2(!valid, "no constraints. Should be True");

    goal.setTheta(fromDegreeToRadiants(90));
    valid = computeOrientation(&start, &goal, 60);
    QVERIFY2(!valid, "no constraints. Should be True");

    goal.setTheta(fromDegreeToRadiants(-90));
    valid = computeOrientation(&start, &goal, 60);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(-45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(180));
    valid = computeOrientation(&start, &goal, 60);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));

    goal.setTheta(fromDegreeToRadiants(-180));
    valid = computeOrientation(&start, &goal, 60);
    QVERIFY2(valid, "no constraints. Should be True");
    QVERIFY2(start.getSteeringAngle() == fromDegreeToRadiants(45), "to reach (1, 1, -) from (0, 0, 0) i should turn of 45°");
    QCOMPARE(start.getTheta(),  fromDegreeToRadiants(-135));
}

}
