#ifndef DUMMYFUNCTION_H
#define DUMMYFUNCTION_H

#include "exploration/evaluationfunction.h"

namespace Exploration{
class DummyFunction : public EvaluationFunction
{

public:
    DummyFunction(uint robotId);
    virtual ~DummyFunction();

    virtual double evaluateFrontier(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                                    int batteryTime, QHash<uint, double> &powerSignalData);

    virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);
};
}

#endif // DUMMYFUNCTION_H
