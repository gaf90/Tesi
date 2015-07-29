#ifndef USERAREACRITERION_H
#define USERAREACRITERION_H

#include "criterion.h"

namespace Exploration{
class UserAreaCriterion : public Criterion
{
public:
    UserAreaCriterion(double weight, uint robotId);
    virtual ~UserAreaCriterion();

    double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                          int batteryTime, QHash<uint, double> &powerSignalData);
    void setPoint(SLAM::Geometry::Point &point);
private:
    uint robotId;
    SLAM::Geometry::Point point;

};
}

#endif // USERAREACRITERION_H
