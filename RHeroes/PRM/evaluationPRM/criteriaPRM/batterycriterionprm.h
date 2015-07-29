#ifndef BATTERYCRITERIONPRM_H
#define BATTERYCRITERIONPRM_H

#include "criterionprm.h"

namespace PRM{
using namespace SLAM;
using namespace SLAM::Geometry;

class BatteryCriterionPRM : public CriterionPRM
{
public:
    BatteryCriterionPRM(double weight, uint robotId);
    virtual ~BatteryCriterionPRM();
    double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);
    void normalize();

private:
    uint robotId;
};


}
#endif // BATTERYCRITERIONPRM_H
