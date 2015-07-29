#ifndef COVERAGEALGORITHM_H
#define COVERAGEALGORITHM_H

#include <QStack>
#include <data/action.h>

namespace PathPlanner{
class CoverageAlgorithm
{
public:
    CoverageAlgorithm();
    virtual ~CoverageAlgorithm();

    QStack<Data::Action *> * doAlgorithm();
};
}

#endif // COVERAGEALGORITHM_H
