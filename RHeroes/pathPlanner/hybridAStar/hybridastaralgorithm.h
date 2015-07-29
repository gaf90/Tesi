#ifndef HYBRIDASTARALGORITHM_H
#define HYBRIDASTARALGORITHM_H

#include "slam/geometry/linesegment.h"
#include "data/pose.h"
#include "pathPlanner/hybridAStar/hybridastaraction.h"
#include "pathPlanner/hybridAStar/hybridastarnode.h"
#include <QRectF>
#include <QSet>
#include <QStack>
#include <QQueue>
#include <QMultiMap>
#include "pathPlanner/abstractaction.h"
#include "slam/map.h"

#define ROBOT P3AT

#define PP_MAX_ITERATIONS   2000

#define PASS_THROUGH_UNKNOWN_AREAS_ALLOWED true
#define DISTANCE_FOR_DESTINATION_POINT_REACHED 0.1 //if reached a point that is distant less than this value to the destination point then we have found a solution
#define ROTATION_WEIGHT (5/M_PI) //needed for the heuristic

#define WHEEL_BASE 0.2480 //width of the robot
#define MAX_SPEED (MAX_SPEED_RH * 0.63)//rad/s
#define WHEEL_RADIUS 0.0980 //meters


#define TOP_SPEED_PERCENTAGE 1
#define ROTATION_SPEED_MAIN_WHEEL_PERCENTAGE 0.7
#define ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE 0.5

#define DELTA_T 0.5
#define CELL_SIDE 0.1
#define P3AT_RADIUS 0.4
#define KENAF_RADIUS 0.2

#define SAFETY_PREFETCH_RADIUS_RH 0.66
#define REACH_AT_LEAST_THE_CLOSEST_POSITION //if defined, reach the closest

namespace PathPlanner{
class HybridAStarAlgorithm
{
public:
    HybridAStarAlgorithm(bool isKenaf);

    struct CellIndex {
        int x, y, theta;
    };

    QStack<AbstractAction *> * computePath(const Data::Pose &startingPose, const Data::Pose &destinationPose,
                                           SLAM::Map *aSlamMap, bool aNeedOrientation = true);
    void printWalls(const QList<SLAM::Geometry::LineSegment *> & walls, QString varName);
    void printStartDestPoints(const Data::Pose &startingPose, const Data::Pose &destinationPose);
    void printPath(QList<PathPlanner::HybridAStarNode> path);
    void printPoints(const QList<Data::Pose> &poses, QString varName);
private:
    QQueue<HybridAStarNode> calculateNodes(const Data::Pose &startingPose, const Data::Pose &destinationPose);
    double heuristicDistance(const Data::Pose &currentPose, const Data::Pose &destinationPose);
    HybridAStarNode * calculateNextNode(HybridAStarNode &oldNode, HybridAStarAction &action);
    bool isAlreadyVisitedCell(HybridAStarNode & node, QSet<CellIndex> & visitedCells);
    bool canReach(const HybridAStarNode & prevNode, const HybridAStarNode & currentNode);
    CellIndex calculateCell(Data::Pose pose);
    QStack<AbstractAction *> *getPoseFromNodes(QQueue<HybridAStarNode> & nodes);

    QStack<AbstractAction *> *getOldStylePoseFromNodes(QQueue<HybridAStarNode> & nodes);

    QList<HybridAStarAction> availableActions;
    QList<Data::Pose> testedPoses;
    SLAM::Map *slamMap;
    bool needOrientation;
    bool isKenaf;

    static const double aStarWeight;

    //remove, only for testing
    void loadWalls();
    void createWalls(QList<SLAM::Geometry::Point> points);
    QList<SLAM::Geometry::LineSegment *> walls;
    bool canFakeReach(Data::Pose startingPose, Data::Pose destinationPose);
};
}
#endif // HYBRIDASTARALGORITHM_H
