#include "astarnodecomparator.h"

namespace PathPlanner{
AStarNodeComparator::AStarNodeComparator()
{
}

AStarNodeComparator::~AStarNodeComparator()
{

}

bool AStarNodeComparator::operator()(const AStarNode *n1, const AStarNode *n2)
{
    return n1->getFValue() < n2->getFValue();
}

}
