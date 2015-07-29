#ifndef ASTARALGORITHM_H
#define ASTARALGORITHM_H

#include <QStack>
#include <QList>
#include "data/action.h"
#include "astarnode.h"
#include "slam/map.h"
#include "slam/geometry/frontier.h"
#include "slam/pathnode.h"
#include "pathPlanner/abstractaction.h"

namespace PathPlanner{
class AStarAlgorithm
{
public:
    explicit AStarAlgorithm(uint identifier, const SLAM::Map *slamMap, double xGoal, double yGoal);
    virtual ~AStarAlgorithm();

    QStack<AbstractAction *> * doAlgorithm();

private:
    AStarNode* chooseANodeToExpand();
    AStarNode* frontierFound(AStarNode *node);
    void createNewNode(const SLAM::PathNode *pathNode, AStarNode *toExpand);
    QStack<AbstractAction *> * computePathToTheGoal(AStarNode *goalFound);
    SLAM::Geometry::Point * computeSafePoint(SLAM::Geometry::Frontier *f, const SLAM::PathNode *node);
    bool foundIntersection(SLAM::Geometry::Point endPoint, const SLAM::PathNode *node);

    bool rectContainsWall(const SLAM::Geometry::LineSegment *wall, const QRectF &rect);
    int numberOfIntersectingSegmentsWithAGivenRectangleBuiltByExpandingAPoint(const SLAM::Geometry::LineSegment *ls, const QRectF &rect);
    void printPath(AStarNode *node);
    void printPlan(AStarNode *actual, AStarNode *next, Data::Action *rotAction, Data::Action *traAction);

    const SLAM::Map *slamMap;
    AStarNode *startNode;
    AStarNode *goalNode;
    const SLAM::PathNode *robotPose;
    SLAM::PathNode *goalPose;
    bool shouldEnd;
    QList<AStarNode *> *closedSet, *openSet;

};
}

#endif // ASTARALGORITHM_H
