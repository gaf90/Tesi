#ifndef TRANSMISSIONPROBABILITYCRITERIONPRM_H
#define TRANSMISSIONPROBABILITYCRITERIONPRM_H

#include "criterionprm.h"

namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;
using namespace Data;

class TransmissionProbabilityCriterionPRM : public CriterionPRM
{
public:
    TransmissionProbabilityCriterionPRM(double weight, uint robotId);
    virtual ~TransmissionProbabilityCriterionPRM();

    double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);
    void normalize();
private:
    double computeAttenuation(const Pose& myPose, const Pose& otherPose, const Frontier *frontier);

    double computePathLoss(double attenuationFactor,const Frontier *frontier,const Map &map,QHash<uint, double> &powerSignalData);
    uint robotId;
};
}
#endif // TRANSMISSIONPROBABILITYCRITERIONPRM_H
