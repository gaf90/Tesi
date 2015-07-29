#include "testrrtnode.h"
#include "pathPlanner/RRT/rrtnode.h"


namespace Test{

using namespace PathPlanner;

TestRRTNode::TestRRTNode(QObject *parent) :
    QObject(parent)
{
}

TestRRTNode::~TestRRTNode()
{

}

void TestRRTNode::testEquals()
{
    RRTNode n1, n2, parent;

    parent.setX(0.0); parent.setY(0.0);
    parent.setTheta(0.0); parent.setSteeringAngle(0.0);
    parent.setValid(true); parent.setParent(NULL);

    n1.setX(1.0); n1.setY(1.0);
    n1.setTheta(0.0); n1.setSteeringAngle(0.0);
    n1.setValid(true); n1.setParent(&parent);

    n2.setX(1.0); n2.setY(1.0);
    n2.setTheta(0.0); n2.setSteeringAngle(0.0);
    n2.setValid(true); n2.setParent(&parent);

    QVERIFY(n1 == n2);
    QVERIFY(n2 == n1);
    n2.setX(0.0);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    n2.setX(1.0); n2.setY(0.0);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    n2.setY(1.0); n2.setTheta(1.0);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    n2.setTheta(0.0); n2.setSteeringAngle(1.0);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    n2.setSteeringAngle(0.0); n2.setValid(false);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    n2.setValid(true);

    RRTNode parent2;
    parent2.setX(1.0); parent2.setY(0.0);
    parent2.setTheta(0.0); parent2.setSteeringAngle(0.0);
    parent2.setValid(true); parent2.setParent(NULL);
    n2.setParent(&parent2);
    QVERIFY(!(n1 == n2));
    QVERIFY(!(n2 == n1));
    qDebug() << "uguale...";

}

void TestRRTNode::testNotEquals()
{
    RRTNode n1, n2, parent;

    parent.setX(0.0); parent.setY(0.0);
    parent.setTheta(0.0); parent.setSteeringAngle(0.0);
    parent.setValid(true); parent.setParent(NULL);

    n1.setX(1.0); n1.setY(1.0);
    n1.setTheta(0.0); n1.setSteeringAngle(0.0);
    n1.setValid(true); n1.setParent(&parent);

    n2.setX(1.0); n2.setY(1.0);
    n2.setTheta(0.0); n2.setSteeringAngle(0.0);
    n2.setValid(true); n2.setParent(&parent);

    QVERIFY(!(n1 != n2));
    QVERIFY(!(n2 != n1));
    n2.setX(0.0);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);
    n2.setX(1.0); n2.setY(0.0);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);
    n2.setY(1.0); n2.setTheta(1.0);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);
    n2.setTheta(0.0); n2.setSteeringAngle(1.0);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);
    n2.setSteeringAngle(0.0); n2.setValid(false);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);
    n2.setValid(true);

    RRTNode parent2;
    parent2.setX(1.0); parent2.setY(0.0);
    parent2.setTheta(0.0); parent2.setSteeringAngle(0.0);
    parent2.setValid(true); parent2.setParent(NULL);
    n2.setParent(&parent2);
    QVERIFY(n1 != n2);
    QVERIFY(n2 != n1);

    qDebug() << "non uguale...";
}

void TestRRTNode::testDistance()
{
    RRTNode n1, n2, parent;

    parent.setX(0.0); parent.setY(0.0);
    parent.setTheta(0.0); parent.setSteeringAngle(0.0);
    parent.setValid(true); parent.setParent(NULL);

    n1.setX(1.0); n1.setY(1.0);
    n1.setTheta(0.0); n1.setSteeringAngle(0.0);
    n1.setValid(true); n1.setParent(&parent);

    n2.setX(1.0); n2.setY(1.0);
    n2.setTheta(0.0); n2.setSteeringAngle(0.0);
    n2.setValid(true); n2.setParent(&parent);

    QVERIFY(EuclideanDistance(n1, n2) == 0);
    QVERIFY(EuclideanDistance(n2, n1) == 0);
    n2.setX(2.0);
    QVERIFY(EuclideanDistance(n1, n2) == 1);
    QVERIFY(EuclideanDistance(n2, n1) == 1);
    QVERIFY(EuclideanDistance(parent, n1) == sqrt(2));
    QVERIFY(EuclideanDistance(n1, parent) == sqrt(2));

    QVERIFY(EuclideanDistance(n1, n1) == 0);

}

QTEST_MAIN(TestRRTNode)
}
