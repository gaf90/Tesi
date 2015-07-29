#ifndef ASTARNODECOMPARATOR_H
#define ASTARNODECOMPARATOR_H

#include "astarnode.h"


namespace PathPlanner{
class AStarNodeComparator
{
public:
    AStarNodeComparator();
    virtual ~AStarNodeComparator();

    bool operator()(const AStarNode *n1, const AStarNode *n2);
};
}
#endif // ASTARNODECOMPARATOR_H
