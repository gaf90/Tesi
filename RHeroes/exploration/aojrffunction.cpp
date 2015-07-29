#include "aojrffunction.h"
#include "shared/config.h"
#include "shared/logger.h"
#include "shared/constants.h"
#include "data/pose.h"
#include <cmath>



namespace Exploration{

using namespace SLAM;
using namespace Geometry;
using namespace Data;

    AOJRFFunction::AOJRFFunction(uint robotId) :
        EvaluationFunction(robotId)
    {

    }

    AOJRFFunction::~AOJRFFunction()
    {

    }

    double AOJRFFunction::evaluateFrontier(
            const Frontier *frontier, const Map &map,
            int batteryTime, QHash<uint, double> &powerSignalData)
    {
        double infoGain = EvaluationFunction::computeInformationGain(*frontier, map);
        double commProb = EvaluationFunction::computeTransmissionProbability(
                    *frontier, map, powerSignalData);

        const Pose *robotPose = map.lastRobotPose(robotId);
        double distance = frontier->centroid().distance(Point(robotPose->getX(), robotPose->getY()));

        ldbg << "evaluation of frontier with centroid <"<<frontier->centroid().x()<<", "<<frontier->centroid().y()<<">"<<endl;
        ldbg << "infoGain = "<<infoGain<<"; commProb = "<<commProb<<";\ndistance = "<<distance<<"; battery = "<<batteryTime<<endl;
        double toRet = (infoGain*commProb)/pow(distance, batteryTime);
        ldbg << "result (infoGain*commProb)/pow(distance, batteryTime) = "<<toRet<<endl;

        return toRet;
    }

    void AOJRFFunction::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command){
        Q_UNUSED(activate) Q_UNUSED(command)
    }


}
