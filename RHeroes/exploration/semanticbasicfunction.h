#ifndef SEMANTICBASICFUNCTION_H
#define SEMANTICBASICFUNCTION_H

#include "evaluationfunction.h"


namespace Exploration{

    /**
      * This class implements the evaluation function used by Stachniss
      * to evaluate the utility of the frontiers using semantic informations.
      */
    class SemanticBasicFunction : public EvaluationFunction
    {

    public:
        SemanticBasicFunction(uint robotId);
        virtual ~SemanticBasicFunction();

        virtual double evaluateFrontier(
                const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                int batteryTime, QHash<uint, double> &powerSignalData);
        virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);

    };
}

#endif // SEMANTICBASICFUNCTION_H
