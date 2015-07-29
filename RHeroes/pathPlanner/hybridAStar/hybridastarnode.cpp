#include "hybridastarnode.h"

namespace PathPlanner{
HybridAStarNode::HybridAStarNode()
{
}
HybridAStarNode::HybridAStarNode(Data::Pose aPose, HybridAStarAction aAction, double aDistance)
{
    pose = aPose;
    action = aAction;
    distance = aDistance;
}

HybridAStarNode::HybridAStarNode(HybridAStarNode *node)
{
    pose = node->getPose();
    action = node->getAction();
    distance = node->getDistance();
}

Data::Pose HybridAStarNode::getPose() const
{
    return pose;
}

HybridAStarAction HybridAStarNode::getAction()
{
    return action;
}

double HybridAStarNode::getDistance()
{
    return distance;
}
}
