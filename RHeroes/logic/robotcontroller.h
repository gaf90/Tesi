#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QStack>
#include <QQueue>
#include <QImage>
#include "data/message.h"
#include "data/action.h"
#include "data/robotstate.h"
#include "data/laserdata.h"
#include "data/odometrydata.h"
#include "data/cameradata.h"
#include "data/statedata.h"
#include "data/sonardata.h"
#include "data/insdata.h"
#include "data/robotstate.h"
#include "data/waypointcommand.h"
#include "slam/timedpose.h"
#include "slam/slammodule.h"
#include <QMutex>
#include "inversekinematic.h"
#include "pathPlanner/abstractaction.h"
#include "pathPlanner/hybridAStar/hybridposeaction.h"
#include "pathPlanner/hybridAStar/hybridastaralgorithm.h"
#include "robot.h"
#include <QTimer>
#include <QTime>
#include "data/sonardata.h"
#include "logic/obstacleavoidancehandler.h"

#define TIMEOUT_MSEC  Config::OBS::obstacle_timeout

#define TRASL_TOL 0.1
#define STALL_NUMBER 25
#define WHEEL_SPEED 0.3

#define UPDATE_STATUS true

/**
 * This class represents the logic for the robot.
 * It is the class that analyzes the data and takes the
 * decisions.
 */
class RobotController : public QObject
{

    //Temporary
    friend class Robot;
    Q_OBJECT
public:

    int wayPointCounter;

    /**
     * Constructor for the RobotController
     */
    explicit RobotController(uint id, QString initialLocation, QString initialRotation, bool isKenaf, QObject *parent = 0);
    /**
     * Destructor for the RobotController.
     */
    virtual ~RobotController();

    /**
      * Return the actual state of the robot.
      * @return the state of the robot.
      */
    Data::RobotState * getActualState();

    void setSlamModule(SLAM::SLAMModule *slam);

    void setStatus(bool enable);

    void handleWheelMotionMessage(const Data::BuddyMessage *buddy);
    void onStateUpdatedHybrid(bool isIdle);
    void onStateUpdatedNormal();
    void handleRobotStall();
    void controlStartRotation(double rotation);
    void controlEndRotation();
    void controlLeftRotation(double lastHeading);
    void controlRightRotation(double lastHeading);
    void controlStartTranslation(const Data::Action &todo);
    void controlEndTranslation();
    void controlTraslationNearSetPoint(double distToCover, double distCovered);
    void controlTraslationTooFar();
    void controlTraslationTooBack();
    void controlTraslationStall(double distCovered, const Data::Action &todo, double distToCover);

private:
   //=== Movement Helper ===//

   void controlRotation(const Data::Action &todo);
   /**
    * This method is invoked to control the translation in order to stop
    * it if the robot has reached the set point.
    * @param the action that is the translation that must be controlled.
    */
   void controlTranslation(const Data::Action &todo);

   //=== Sensor Data Handler ===//

   /**
    * This method handles the data received from the camera.
    * @param camera the data obtained from the camera.
    */
   void handleCameraData(const Data::CameraData &camera);
   /**
    * This method handles the data received from the state of the robot.
    * @param state the data obtained from the state.
    */
   void handleStateData(const Data::StateData &state);
   /**
    * This method handles the data received from the sonar of the robot.
    * @param sonar the data obtained from the sonar.
    */

   void handleWirelessData(const Data::Message &data);

   void notifyAfterWaypoint();

   void saveRobotState();

   //PRM
   bool previousDistancePositive;
   //

signals:
    /**
     * This signal sends a command to the connectors.
     * @param command to send to the connectors.
     */
    void sigDriverMessageRobot(const Data::Message &command);
    /**
     * This signal is emitted to notify that the state has been updated.
     */
    void sigRobotStateUpdated();

    void sigWirelessMessageRCM(const Data::Message &message);

    //TMP signal

    void sigChangeStateExplorationRCM(bool restart);

    void sigChangeStatePathPlanningRCM(bool restart);

    void sigCameraDataRCM(const Data::CameraData &camera, const Data::Pose &pose);

    void sigRecomputePathRCM(const Data::Pose);

    void sigCleanBadFrontierRCM();

    void sigHandleBadFrontierRCM(const Data::Pose);

    void sigChangeMovementType(int);

public slots:
    /**
     * This slot is invoked to handle the state update.
     */
    void onStateUpdated();
    /**
     * This slot is invoked to handle the message from a sensor.
     */
    void onSensorData(const Data::Message &data);

    void updatePose(const Data::Pose &pose);

    void setStreamImageFlag(bool stream);

    void handleWaypoint(const Data::WaypointCommand *waypoint);

    void onNewRobotPose(SLAM::TimedPose pose);

    void onStopRobotForPlanning();

    void onRobotNotReachable();

    void onFrontierToReachRCM(const Data::Pose pose);

    void onPerformActionRCM(PathPlanner::AbstractAction *action);

    void onNoFrontierAvailableRCM();

    void onHandleBadFrontierRCM(Data::Pose);

    void onPointToReachRCM(double x, double y);


    void setActionStartTimestamp(int value);
    void setControlType(int type);
    void setLastSonarData(Data::SonarData sonar);
    void stopRobot(bool saveState);
    void moveRobot(double angle1, double translation, double angle2);
    void sendWheelMessage(double leftSpeed, double rightSpeed);

private slots:

    void onRestartExploration();
    void onRecomputePath(const Data::Pose &actualFrontier);
    void onObstacleStart();
    void onTimeoutObstacle();
    void onTimeoutTeleoperation();
    void onFrontierReached();

private:
    bool poseReached(const Data::Pose &pose);

private:

    enum typeMovementEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
    enum controlTypeEnum {HYBRID,NORMAL};
    controlTypeEnum controlRobotType;
    enum robotTypeEnum{P3AT,KENAF,OTHER};
    robotTypeEnum robotType;
    enum statusEnum{ON,OFF};
    int sonarStatus;
    statusEnum teleoperationStatus;
    enum robotAroundEnum{NEAR_TO_POSE,FAR_TO_POSE};
    robotAroundEnum robotPoseCheck;


    bool isExplorationEnabled;
    bool isPathPlannerEnabled;
    bool isNewMovement;
    bool streamImage;
    bool isNotificationNeeded, haveReceivedWaypoint;

    uint robotId;

    int previousBattery;
    double previousHeading;
    double previousX;
    double previousY;
    int waitTime;
    int stallCounter;

    double actionStartTimestamp;


    QQueue<Data::Action *> *normalActionQueue;
    QQueue<PathPlanner::HybridPoseAction *> *hybridActionQueue;


    Data::RobotState *pastState;
    Data::RobotState *actualState;
    Data::SonarData lastFrontSonarData;
    Data::Pose pastHybridPoseToReach, previousPose;
    Data::Pose *actualFrontier;
    Data::Pose *oldFrontier;
    Data::WaypointCommand *actualWaypoint;

    QMutex *streamMutex;
    InverseKinematic *inverseKinematicModule;
    QTimer *obstacleAvoidanceTimer,*randomActionTimer, *teleOperationTimer;
    QTime crono, frontierTime;

    SLAM::SLAMModule *slam;
    PathPlanner::HybridPoseAction *goalToRecompute;

    ObstacleAvoidance *obstacleAvoidance;

    QString space;
};

#endif // ROBOTCONTROLLER_H
