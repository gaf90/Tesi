#include "informationgaincriterionprm_mcdm.h"
#include "criterianameprm.h"
#include "shared/config.h"

namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

InformationGainCriterionPRM_MCDM::InformationGainCriterionPRM_MCDM(double weight) :
    CriterionPRM(INFORMATION_GAIN_PRM, weight, true)

{
}

InformationGainCriterionPRM_MCDM::~InformationGainCriterionPRM_MCDM()
{

}

double InformationGainCriterionPRM_MCDM::evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                                             int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(map) Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)

            QList<double> length;

    foreach(PRMPath p, paths){
        double value=frontier->length();
        length.append(value);
    }
    insertEvaluation(frontier, length);
    return 0.0;
}

}
