#include "semanticbasicfunction.h"

using namespace SLAM;
using namespace Geometry;

namespace Exploration {
    SemanticBasicFunction::SemanticBasicFunction(uint robotId) :
        EvaluationFunction(robotId)
    {
    }


    SemanticBasicFunction::~SemanticBasicFunction()
    {
    }

    double SemanticBasicFunction::evaluateFrontier(
            const Frontier *frontier, const Map &map, int batteryTime,
            QHash<uint, double> &powerSignalData)
    {
        Q_UNUSED(frontier) Q_UNUSED(map) Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)
        return 0.0;
    }
    void SemanticBasicFunction::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command)
    {
        Q_UNUSED(activate) Q_UNUSED(command)
    }
}
