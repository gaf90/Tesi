#include "userdirectioncriterion.h"
#include "criteriaName.h"
#include "exploration/explorationconstants.h"


namespace Exploration{
    using namespace SLAM::Geometry;

UserDirectionCriterion::UserDirectionCriterion(double weight) :
    Criterion(USER_DIRECTION, weight, true)
{
}

UserDirectionCriterion::~UserDirectionCriterion()
{

}

double UserDirectionCriterion::evaluate(const Frontier *frontier, const SLAM::Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(map) Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)

    double fvalue = alphaX * frontier->centroid().x() + alphaY * frontier->centroid().y();
    insertEvaluation(frontier, fvalue);
    return fvalue;
}

void UserDirectionCriterion::setAlphaX(double alphaX)
{
    this->alphaX = alphaX;
}

void UserDirectionCriterion::setAlphaY(double alphaY)
{
    this->alphaY = alphaY;
}

}


