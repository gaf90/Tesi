#ifndef CRITERIONCOMPARATORPRM_H
#define CRITERIONCOMPARATORPRM_H

#include "criterionprm.h"

namespace PRM{

using namespace SLAM::Geometry;

class CriterionComparatorPRM
{
public:
    explicit CriterionComparatorPRM(Frontier* f, int id);
    bool operator() (const CriterionPRM* c1, CriterionPRM* c2);

private:
    Frontier* frontier;
    int idPath;
};

}
#endif // CRITERIONCOMPARATORPRM_H
