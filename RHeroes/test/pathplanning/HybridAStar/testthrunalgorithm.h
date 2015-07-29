#ifndef TESTTHRUNALGORITHM_H
#define TESTTHRUNALGORITHM_H

#define MAX_X 50
#define MAX_Y 50
#define N_WALLS 100
#define MIN_LENGTH_WALL 3
#define MAX_LENGTH_WALL 6
#define UNKNOWN_AREAS 3
#define MIN_SIDE_DIMENSION_UNKNOWN_AREA 2
#define MAX_SIDE_DIMENSION_UNKNOWN_AREA 5

#include "pathPlanner/hybridAStar/hybridastaralgorithm.h"
#include "pathPlanner/hybridAStar/hybridastaraction.h"
#include "pathPlanner/hybridAStar/hybridastarnode.h"

namespace Test{
class TestThrunAlgorithm
{
public:
    TestThrunAlgorithm(bool isKenaf);
    void startSimulation();

private:
    void createUnknownAreas();
    void createRandomWalls();
    void loadWalls();
    void createWalls(QList<SLAM::Geometry::Point> points);

    double fRand(double fMin, double fMax);

    void printUnknownAreas();
    void printWalls();
    void printStartDestPoints(Data::Pose & startingPose, Data::Pose & destinationPose);
    void printPath();

    PathPlanner::HybridAStarAlgorithm algorithm;
    QList<SLAM::Geometry::LineSegment *> walls;
    QList<QRectF> unknownAreas;
    QList<PathPlanner::HybridAStarNode> path;    
};
}
#endif // TESTTHRUNALGORITHM_H
