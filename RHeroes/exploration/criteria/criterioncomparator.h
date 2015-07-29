#ifndef CRITERIONCOMPARATOR_H
#define CRITERIONCOMPARATOR_H

#include <QObject>
#include "criterion.h"
#include "slam/geometry/point.h"

namespace Exploration{
class CriterionComparator
{

public:
    explicit CriterionComparator(SLAM::Geometry::Frontier *p);
    virtual ~CriterionComparator();
    bool operator()(const Criterion *c1, const Criterion *c2 );

private:
    SLAM::Geometry::Frontier *p;
    
};
}
#endif // CRITERIONCOMPARATOR_H
