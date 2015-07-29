#include "hybridastaraction.h"
#include "hybridastaralgorithm.h"

namespace PathPlanner{
HybridAStarAction::HybridAStarAction()
{
}

HybridAStarAction::HybridAStarAction(double aVr, double aVl)
{
    vr = aVr;
    vl = aVl;
}

double HybridAStarAction::getVr()
{
    return vr;
}

double HybridAStarAction::getVl()
{
    return vl;
}
}

double PathPlanner::HybridAStarAction::getTimeEstimate()
{
    return DELTA_T;
}
