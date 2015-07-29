#include "batterycriterion.h"
#include "criteriaName.h"
#include "data/action.h"
#include "shared/utilities.h"

namespace Exploration{

using namespace SLAM::Geometry;
using namespace Data;

BatteryCriterion::BatteryCriterion(double weight, uint robotId) :
    Criterion(BATTERY, weight, true), robotId(robotId)
{
}

BatteryCriterion::~BatteryCriterion()
{

}

double BatteryCriterion::evaluate(const Frontier *frontier, const SLAM::Map &map,
              int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(frontier) Q_UNUSED(map) Q_UNUSED(powerSignalData)

    SLAM::Geometry::Point myPoint(map.lastRobotPose(robotId)->getX(), map.lastRobotPose(robotId)->getY());
    SLAM::Geometry::Point fCentr = frontier->centroid();
    //store the evaluation
    double distance = myPoint.distance(fCentr);

    Action trasl;
    trasl.setType(Action::Translation);
    trasl.setValue(distance);

    Action rotat;
    rotat.setType(Action::Rotation);
    const Pose *pose = map.lastRobotPose(robotId);
    const Pose toReach(frontier->centroid().x(), frontier->centroid().y(), 0.0);
    double angle2 = computeRotationFromPoses(*pose, toReach);
    rotat.setValue(fromRadiantToDegree(angle2));
    double traslTime = trasl.getTimeEstimate();
    double rotTime = rotat.getTimeEstimate();
    double timeRequired = traslTime+rotTime;
    double remainingBattery = batteryTime - timeRequired;
    insertEvaluation(frontier,remainingBattery);
    return remainingBattery;
}



}
