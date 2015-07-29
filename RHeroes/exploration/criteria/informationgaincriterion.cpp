#include "informationgaincriterion.h"
#include "criteriaName.h"

namespace Exploration{

using namespace SLAM::Geometry;

InformationGainCriterion::InformationGainCriterion(double weight) :
    Criterion(INFORMATION_GAIN, weight, true)

{
}


InformationGainCriterion::~InformationGainCriterion()
{

}

double InformationGainCriterion::evaluate(const Frontier *frontier, const SLAM::Map &map,
              int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(map) Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)



    double length = frontier->length();
    insertEvaluation(frontier, length);
    return length;
}

}
