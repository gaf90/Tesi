#include "rrtnode.h"

namespace PathPlanner{

    RRTNode::RRTNode(double x, double y) :
        x(x), y(y), theta(0), steeringAngle(0), parent(NULL),
        reachable(new QSet<RRTNode *>()), valid(true)
    {
    }

    RRTNode::RRTNode() :
        x(0), y(0), theta(0), steeringAngle(0), parent(NULL),
        reachable(new QSet<RRTNode *>()), valid(true)
    {
    }

    RRTNode::~RRTNode()
    {
        delete reachable;
    }

    double RRTNode::getX() const
    {
        return x;
    }
    double RRTNode::getY() const
    {
        return y;
    }
    double RRTNode::getTheta() const
    {
        return theta;
    }
    double RRTNode::getSteeringAngle() const
    {
        return steeringAngle;
    }
    RRTNode * RRTNode::getParent() const
    {
        return parent;
    }
    bool RRTNode::isValid() const
    {
        return valid;
    }

    QSetIterator<RRTNode *> * RRTNode::getReachableIterator() const
    {
        QSetIterator<RRTNode *> * toRet = new QSetIterator<RRTNode *>(*reachable);
        return toRet;
    }

    void RRTNode::setX(double x)
    {
        this->x = x;
    }

    void RRTNode::setY(double y)
    {
        this->y = y;
    }

    void RRTNode::setTheta(double theta)
    {
        this->theta = theta;
    }

    void RRTNode::setSteeringAngle(double steeringAngle)
    {
        this->steeringAngle = steeringAngle;
    }

    void RRTNode::setParent(RRTNode * parent)
    {
        this->parent = parent;
    }


    void RRTNode::setValid(bool valid)
    {
        this->valid = valid;
    }

    void RRTNode::addReachable(RRTNode * toAdd)
    {
        reachable->insert(toAdd);
    }

}
