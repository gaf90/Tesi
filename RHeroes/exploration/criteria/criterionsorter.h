#ifndef CRITERIONSORTER_H
#define CRITERIONSORTER_H

#include "criterion.h"

namespace Exploration{
    class CriterionSorter
    {
    public:
        CriterionSorter();
        virtual ~CriterionSorter();

        bool operator()(const Criterion &c1, const Criterion &c2);
    };
}

#endif // CRITERIONSORTER_H
