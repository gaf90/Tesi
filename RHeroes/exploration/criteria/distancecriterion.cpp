#include "distancecriterion.h"
#include "criteriaName.h"
namespace Exploration{

using namespace SLAM::Geometry;

DistanceCriterion::DistanceCriterion(double weight, uint robotId)
    : Criterion(DISTANCE, weight, false), robotId(robotId)
{
}

DistanceCriterion::~DistanceCriterion()
{

}

double DistanceCriterion::evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData)
{

    Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)
    //compute the distance robot-frontier
    SLAM::Geometry::Point myPoint(map.lastRobotPose(robotId)->getX(), map.lastRobotPose(robotId)->getY());
    SLAM::Geometry::Point fCentr = frontier->centroid();
    //store the evaluation
    double distance = myPoint.distance(fCentr);
    insertEvaluation(frontier, distance);
    return distance;
}

}
