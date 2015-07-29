#include "rrtnodecomparator.h"
namespace PathPlanner{
RRTNodeComparator::RRTNodeComparator(RRTNode * ref) :
    ref(ref)
{

}

RRTNodeComparator::~RRTNodeComparator()
{

}

bool RRTNodeComparator::operator()(const RRTNode *n1, const RRTNode *n2)
{
    double dist1 = EuclideanDistance(*n1, *ref);
    double dist2 = EuclideanDistance(*n2, *ref);
    return dist1<=dist2;
}

}
