#include "criterionsorter.h"
namespace Exploration{
CriterionSorter::CriterionSorter()
{
}

CriterionSorter::~CriterionSorter()
{

}

bool CriterionSorter::operator ()(const Criterion &c1, const Criterion &c2)
{
    return c1.getEvaluation() < c2.getEvaluation();
}

}
