#ifndef INFORMATIONGAINCRITERIONPRM_MCDM_H
#define INFORMATIONGAINCRITERIONPRM_MCDM_H

#include "criterionprm.h"
#include "testPRM/testevaluationprm.h"


namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

class InformationGainCriterionPRM_MCDM : public CriterionPRM
{
    friend class TestPRM::TestEvaluationPRM;
public:
    InformationGainCriterionPRM_MCDM(double weight);
    virtual ~InformationGainCriterionPRM_MCDM();

    double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                  int batteryTime, QHash<uint, double> &powerSignalData);

};
}



#endif // INFORMATIONGAINCRITERIONPRM_MCDM_H
