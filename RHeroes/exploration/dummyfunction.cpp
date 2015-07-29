#include "dummyfunction.h"

namespace Exploration{

using namespace Data;
using namespace SLAM::Geometry;

DummyFunction::DummyFunction(uint robotId) :
    EvaluationFunction(robotId)
{
}

DummyFunction::~DummyFunction()
{

}

double DummyFunction::evaluateFrontier(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                                int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)
    double infoGain = EvaluationFunction::computeInformationGain(*frontier, map);
    ldbg << "infoGain: " << infoGain << endl;
    ldbg << "try to get my pose... (Call MLADEN Function!)" << endl;

    ldbg << map.knownRobots() << " " << robotId << endl;

    const Pose *robotPose = map.lastRobotPose(robotId);
    ldbg << "robotPose obtained... (Well done, MLADEN!)" << endl;
    double distance = frontier->centroid().distance(Point(robotPose->getX(), robotPose->getY()));
    ldbg << "distance = " << distance << endl;
    return infoGain / distance;
}

void DummyFunction::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command){
    Q_UNUSED(activate) Q_UNUSED(command)
}

}
