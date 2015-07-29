#ifndef HYBRIDASTARNODE_H
#define HYBRIDASTARNODE_H

#include "data/pose.h"
#include "pathPlanner/hybridAStar/hybridastaraction.h"

namespace PathPlanner
{
class HybridAStarNode
{
public:
    HybridAStarNode();
    HybridAStarNode(Data::Pose pose, HybridAStarAction action, double distance);
    HybridAStarNode(HybridAStarNode *node);
    Data::Pose getPose() const;
    HybridAStarAction getAction();
    double getDistance();

private:
    Data::Pose pose;
    HybridAStarAction action;
    double distance;
};
}
#endif // HYBRIDASTARNODE_H
