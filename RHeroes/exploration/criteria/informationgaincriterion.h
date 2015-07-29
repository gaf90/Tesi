#ifndef INFORMATIONGAINCRITERION_H
#define INFORMATIONGAINCRITERION_H

#include "criterion.h"

namespace Exploration{
class InformationGainCriterion : public Criterion
{
public:
    InformationGainCriterion(double weight);
    virtual ~InformationGainCriterion();

    double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);
};
}

#endif // INFORMATIONGAINCRITERION_H
