#ifndef AOJRFFUNCTION_H
#define AOJRFFUNCTION_H

#include "evaluationfunction.h"

namespace Exploration {
    /**
      * This class implements the evaluation function used by Amsterdam-Oxford
      * Joint Rescue Force to evaluate the utility of the frontiers.
      */
    class AOJRFFunction : public EvaluationFunction
    {

    public:
        /**
         * Constructor
         * @param robotId the identifier of the robot.
         */
        AOJRFFunction(uint robotId);
        virtual ~AOJRFFunction();

        virtual double evaluateFrontier(
                const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                int batteryTime, QHash<uint, double> &powerSignalData);

        virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);

    };
}

#endif // AOJRFFUNCTION_H
