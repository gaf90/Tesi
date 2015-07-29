#include "userareacriterion.h"
#include "criteriaName.h"
#include "slam/geometry/point.h"

namespace Exploration{

using namespace SLAM::Geometry;

UserAreaCriterion::UserAreaCriterion(double weight, uint robotId) :
    Criterion(USER_AREA, weight, false), robotId(robotId)
{
}

UserAreaCriterion::~UserAreaCriterion()
{

}

void UserAreaCriterion::setPoint(SLAM::Geometry::Point &point)
{
    this->point = point;
}

double UserAreaCriterion::evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                          int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(map) Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)

    Point fPoint = frontier->centroid();
    double distance = point.distance(fPoint);
    insertEvaluation(frontier, distance);
    return distance;
}

}
