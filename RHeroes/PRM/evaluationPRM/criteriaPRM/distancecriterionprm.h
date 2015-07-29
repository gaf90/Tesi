#ifndef DISTANCECRITERIONPRM_H
#define DISTANCECRITERIONPRM_H
#include "criterionprm.h"

namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

class DistanceCriterionPRM : public CriterionPRM
{
public:
    DistanceCriterionPRM(double weight, uint robotId);
    virtual ~DistanceCriterionPRM();

    double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData);
protected:
    uint robotId;
private:
    double pathLength(PRMPath path);
};
}

#endif // DISTANCECRITERIONPRM_H
