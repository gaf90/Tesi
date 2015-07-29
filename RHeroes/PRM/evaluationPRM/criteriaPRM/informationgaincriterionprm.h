#ifndef INFORMATIONGAINCRITERIONPRM_H
#define INFORMATIONGAINCRITERIONPRM_H

#include "criterionprm.h"
#include "testPRM/testevaluationprm.h"

namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

class InformationGainCriterionPRM : public CriterionPRM
{
    friend class TestPRM::TestEvaluationPRM;
public:
    InformationGainCriterionPRM(double weight);
    virtual ~InformationGainCriterionPRM();

    double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);

private:
    QList<Frontier> visibleFrontiers(Point point,const Map &map);
    double calculateVisibleLength(Point p, Frontier f);
};
}


#endif // INFORMATIONGAINCRITERIONPRM_H
