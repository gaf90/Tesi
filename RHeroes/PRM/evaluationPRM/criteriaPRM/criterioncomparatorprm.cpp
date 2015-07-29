#include "criterioncomparatorprm.h"

namespace PRM{


CriterionComparatorPRM::CriterionComparatorPRM(Frontier* f, int id): frontier(f), idPath(id){

}

bool CriterionComparatorPRM::operator() (const CriterionPRM* c1, CriterionPRM* c2){
    QList<double> values1= c1->getEvaluation(frontier);
    QList<double> values2= c2->getEvaluation(frontier);
    return values1.at(idPath)<values2.at(idPath);
}

}
