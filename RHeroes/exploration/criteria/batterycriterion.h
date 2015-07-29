#ifndef BATTERYCRITERION_H
#define BATTERYCRITERION_H

#include "criterion.h"

namespace Exploration{
class BatteryCriterion : public Criterion
{
public:
    BatteryCriterion(double weight, uint robotId);
    virtual ~BatteryCriterion();

    double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);
    void normalize();

private:
    uint robotId;
};
}

#endif // BATTERYCRITERION_H
