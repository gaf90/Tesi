#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "pose.h"
#include <QString>
#include <QMutex>
#include <QHash>

namespace Data {

class RobotState
{
public:
    RobotState(double aRightSpeed, double aLeftSpeed, double battery);
    RobotState(
            const Pose &aPose, double timestamp, double aRightSpeed,
            double aLeftSpeed, double battery);
    RobotState(const RobotState &aRobotState);
    virtual ~RobotState();
    const Pose &getPose() const;
    void setPose(const Pose &pose);

    double getTimestamp() const;
    void setTimestamp(double timestamp);

    double getRightSpeed() const;
    double getLeftSpeed() const;
    int getBattery() const;
    QHash<uint, double> * getSignalPowerData() const;

    void setRightSpeed(double aRightSpeed);
    void setLeftSpeed(double aLeftSpeed);
    void setBattery(int aBattery);
    void setSignalPowerData(QHash<uint, double> *signalPowerData);
    void setStall(bool stall);

    void clearSignalPowerMap();
    void insertSignalData(uint key, double value);
    void setPowerDataGathered();
    int getNumberOfPowerDataGathered() const;
    /**
      * This method tell if the robot is stopped
      * or if its wheels are moving.
      * @return <b>true</b> if the wheels have a speed < of 0.1 in absolute value; <b>false</b> otherwise
      */
    bool isIdle() const;

    /**
      * This method should return true if the robot is in the same pose for a certain time
      * while its wheels are moving.
      * @return <b>true</b> if the robot wheels are turning but the robot does not moves; <b>false</b> otherwise.
      */
    bool isStall() const;

    QString toString() const;

    bool isExplorationEnabled();
    bool isWaypointControl();
    bool isManualControl();
    bool isSonarEnabled();

    void setExplorationEnabled(bool enabled);
    void setWaypointControl(bool enabled);
    void setManualControl(bool enabled);
    void setSonarEnabled(bool enabled);

private:
    Pose pose;
    double timestamp;

    double rightSpeed;
    double leftSpeed;
    int battery;
    bool stall;

    QHash<uint, double> * signalPowerData;
    bool powerDataSet;
    QMutex *mutex;

    bool explorationEnabled,waypointControl, manualControl;
    bool sonarEnabled;
};

}

#endif // ROBOTSTATE_H
