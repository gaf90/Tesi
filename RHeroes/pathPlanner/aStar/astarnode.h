#ifndef ASTARNODE_H
#define ASTARNODE_H

#include <QList>
#include "slam/pathnode.h"
#include "exploration/explorationconstants.h"

namespace PathPlanner{
class AStarNode
{
public:
    AStarNode();
    virtual ~AStarNode();

    double getGValue() const;
    double getHValue() const;
    double getFValue() const;
    AStarNode* getParent() const;
    QList<AStarNode *> *getChildren() const;
    const SLAM::PathNode* getPose() const;
    bool getOwnPose() const;

    void setGValue(double gvalue);
    void setHValue(double hvalue);
    void setParent(AStarNode* parent);
    void addChild(AStarNode* child) const;
    void setPose(SLAM::PathNode *pose);
    void setOwnPose(bool ownPose);

    inline bool operator==(const AStarNode *node) {

        ldbg << "Equality test invoked" << endl;
        double distance = this->distance(node);
        return distance < MAX_FRONT_RADIUS;

    }

    inline bool operator!=(const AStarNode *node) {
        return !(this == node);
    }

    inline double distance(const AStarNode *node) const {

        SLAM::Geometry::Point me(pose->getX(), pose->getY());
        SLAM::Geometry::Point other(node->getPose()->getX(), node->getPose()->getY());

        double distance = me.distance(other);
        return distance;

    }

private:
    double gvalue;
    double hvalue;
    AStarNode *parent;
    QList<AStarNode *> *children;
    SLAM::PathNode *pose;
    bool ownPose;

};

}

#endif // ASTARNODE_H
