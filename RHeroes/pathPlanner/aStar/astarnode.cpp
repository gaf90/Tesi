#include "astarnode.h"
#include "slam/geometry/point.h"

namespace PathPlanner{

using namespace SLAM::Geometry;

AStarNode::AStarNode() :
    children(new QList<AStarNode *>()), ownPose(false)
{
}

AStarNode::~AStarNode()
{
    while(!children->isEmpty()){
        delete children->last();
        children->removeLast();
    }
    delete children;

    if(ownPose)
        delete pose;
}

double AStarNode::getGValue() const{
    return gvalue;
}

double AStarNode::getHValue() const
{
    return hvalue;
}

double AStarNode::getFValue() const
{
    return gvalue+hvalue;
}

AStarNode* AStarNode::getParent() const
{
    return parent;
}

QList<AStarNode *>* AStarNode::getChildren() const
{
    return children;
}

const SLAM::PathNode* AStarNode::getPose() const
{
    return pose;
}

bool AStarNode::getOwnPose() const
{
    return ownPose;
}

void AStarNode::setGValue(double gvalue)
{
    this->gvalue = gvalue;
}

void AStarNode::setHValue(double hvalue)
{
    this->hvalue = hvalue;
}

void AStarNode::setParent(AStarNode* parent)
{
    this->parent = parent;
}

void AStarNode::addChild(AStarNode* child) const
{
    children->append(child);
}

void AStarNode::setPose(SLAM::PathNode *pose)
{
    this->pose = pose;
}

void AStarNode::setOwnPose(bool ownPose){
    this->ownPose = ownPose;
}







}
