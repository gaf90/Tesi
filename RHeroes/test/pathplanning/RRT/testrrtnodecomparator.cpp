#include "testrrtnodecomparator.h"
#include "pathPlanner/RRT/rrtnodecomparator.h"
namespace Test {

using namespace PathPlanner;

TestRRTnodeComparator::TestRRTnodeComparator(QObject *parent) :
    QObject(parent)
{
}
TestRRTnodeComparator::~TestRRTnodeComparator()
{

}

void TestRRTnodeComparator::testCompare()
{
    RRTNode ref;
    ref.setX(0.0);
    ref.setY(0.0);

    RRTNode n1, n2;
    n1.setX(1.0);
    n1.setY(0.0);

    n2.setX(1.0);
    n2.setY(1.0);

    RRTNodeComparator comparator(&ref);
    QVERIFY(comparator(&n1, &n2));
    QVERIFY(!(comparator(&n2, &n1)));


}
}
