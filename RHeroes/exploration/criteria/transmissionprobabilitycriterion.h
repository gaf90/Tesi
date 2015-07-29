#ifndef TRANSMISSIONPROBABILITYCRITERION_H
#define TRANSMISSIONPROBABILITYCRITERION_H

#include "criterion.h"

namespace Exploration{
class TransmissionProbabilityCriterion : public Criterion
{
public:
    TransmissionProbabilityCriterion(double weight, uint robotId);
    virtual ~TransmissionProbabilityCriterion();

    double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);
    void normalize();
private:
    double computeAttenuation(const Data::Pose& myPose, const Data::Pose& otherPose, const SLAM::Geometry::Frontier *frontier);

    double computePathLoss(double attenuationFactor,const SLAM::Geometry::Frontier *frontier,const SLAM::Map &map,QHash<uint, double> &powerSignalData);
    uint robotId;
};
}

#endif // TRANSMISSIONPROBABILITYCRITERION_H
