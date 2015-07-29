#include "robotstate.h"
#include <cmath>

namespace Data {

RobotState::RobotState(double aRightSpeed, double aLeftSpeed, double battery) :
    pose(Pose(0.0, 0.0, 0.0)), rightSpeed(aRightSpeed), leftSpeed(aLeftSpeed),
    battery(battery), stall(false), signalPowerData(new QHash<uint, double>()),
    powerDataSet(false), mutex(new QMutex())
{

}

RobotState::RobotState(
        const Pose &aPose, double timestamp, double aRightSpeed, double aLeftSpeed, double battery):
    pose(aPose), timestamp(timestamp), rightSpeed(aRightSpeed), leftSpeed(aLeftSpeed),
    battery(battery), stall(false), signalPowerData(new QHash<uint, double>()),
    powerDataSet(false), mutex(new QMutex())
{

}

RobotState::RobotState(const RobotState &aRobotState) :
    pose(aRobotState.getPose()),
    timestamp(aRobotState.getTimestamp()),
    rightSpeed(aRobotState.getRightSpeed()),
    leftSpeed(aRobotState.getLeftSpeed()),
    battery(aRobotState.getBattery()),
    stall(aRobotState.isStall()),
    signalPowerData(aRobotState.getSignalPowerData()),
    mutex(new QMutex())
{

}

RobotState::~RobotState()
{
    delete mutex;
    if(signalPowerData != NULL)
        delete signalPowerData;
}

const Pose &RobotState::getPose() const
{
    mutex->lock();
    const Pose *p = &pose;
    mutex->unlock();
    return *p;
}


void RobotState::setPose(const Pose &pose) {
    mutex->lock();
    this->pose = pose;
    mutex->unlock();
}

double RobotState::getTimestamp() const
{
    mutex->lock();
    double d = timestamp;
    mutex->unlock();
    return d;
}

void RobotState::setTimestamp(double timestamp)
{
    mutex->lock();
    this->timestamp = timestamp;
    mutex->unlock();
}

double RobotState::getRightSpeed() const
{
    mutex->lock();
    double d = rightSpeed;
    mutex->unlock();
    return d;
}

double RobotState::getLeftSpeed() const
{
    mutex->lock();
    double d = leftSpeed;
    mutex->unlock();
    return d;
}

void RobotState::setRightSpeed(double aRightSpeed)
{
    mutex->lock();
    rightSpeed = aRightSpeed;
    mutex->unlock();
}

void RobotState::setLeftSpeed(double aLeftSpeed)
{
    mutex->lock();
    leftSpeed = aLeftSpeed;
    mutex->unlock();
}

void RobotState::setStall(bool stall)
{
    mutex->lock();
    this->stall = stall;
    mutex->unlock();
}

int RobotState::getBattery() const
{
    mutex->lock();
    int i = battery;
    mutex->unlock();
    return i;
}

void RobotState::setBattery(int aBattery)
{
    mutex->lock();
    battery = aBattery;
    mutex->unlock();
}

bool RobotState::isStall() const
{
    mutex->lock();
    bool toRet = stall;
    mutex->unlock();
    return toRet;
}

QString RobotState::toString() const
{
    mutex->lock();
    QString s = QString().sprintf(
                "x: %f02, y: %f02, th: %f02, r_speed: %f02, l_speed: %f02",
                pose.getX(), pose.getY(), pose.getTheta(), rightSpeed,
                leftSpeed);
    mutex->unlock();
    return s;
}

QHash<uint, double> * RobotState::getSignalPowerData() const
{
    QHash<uint, double> *toRet =  new QHash<uint, double>();
    mutex->lock();
    foreach(uint key, signalPowerData->keys())
        toRet->insert(key, signalPowerData->value(key));
    mutex->unlock();
    return toRet;
}

void RobotState::setSignalPowerData(QHash<uint, double> *signalPowerData)
{
    mutex->lock();
    this->signalPowerData = signalPowerData;
    mutex->unlock();
}

void RobotState::clearSignalPowerMap()
{
    mutex->lock();
    signalPowerData->clear();
    powerDataSet = false;
    mutex->unlock();

}

void RobotState::insertSignalData(uint key, double value)
{
    mutex->lock();
    signalPowerData->insert(key, value);
    mutex->unlock();
}

void RobotState::setPowerDataGathered()
{
    mutex->lock();
    powerDataSet = true;
    mutex->unlock();
}

int RobotState::getNumberOfPowerDataGathered() const
{
    int toRet = 0;
    mutex->lock();
    toRet = signalPowerData->size();
    mutex->unlock();

    return toRet;

}

bool RobotState::isIdle() const
{
    bool toRet = false;
    mutex->lock();
    toRet = fabs(leftSpeed) < 0.05 && fabs(rightSpeed) < 0.05;
    mutex->unlock();
    return toRet;
}

bool RobotState::isExplorationEnabled()
{
    bool toRet;
    mutex->lock();
    toRet = explorationEnabled;
    mutex->unlock();
    return toRet;
}

bool RobotState::isWaypointControl()
{
    bool toRet;
    mutex->lock();
    toRet = waypointControl;
    mutex->unlock();
    return toRet;
}

bool RobotState::isManualControl()
{
    bool toRet;
    mutex->lock();
    toRet = manualControl;
    mutex->unlock();
    return toRet;
}

bool RobotState::isSonarEnabled()
{
    bool toRet;
    mutex->lock();
    toRet = sonarEnabled;
    mutex->unlock();
    return toRet;
}

void RobotState::setExplorationEnabled(bool enabled)
{
    mutex->lock();
    explorationEnabled = enabled;
    mutex->unlock();
}

void RobotState::setWaypointControl(bool enabled)
{
    mutex->lock();
    waypointControl = enabled;
    mutex->unlock();
}

void RobotState::setManualControl(bool enabled)
{
    mutex->lock();
    manualControl = enabled;
    mutex->unlock();
}

void RobotState::setSonarEnabled(bool enabled)
{
    mutex->lock();
    sonarEnabled = enabled;
    mutex->unlock();
}
}
