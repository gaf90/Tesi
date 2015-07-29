#ifndef DISTANCECRITERION_H
#define DISTANCECRITERION_H

#include "criterion.h"


namespace Exploration{
class DistanceCriterion : public Criterion
{
public:
    DistanceCriterion(double weight, uint robotId);
    virtual ~DistanceCriterion();

    double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData);
protected:
    uint robotId;

};
}

#endif // DISTANCECRITERION_H
