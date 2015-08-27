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

#define TELEOPERATION_TIMEOUT_MSEC  5000

#define ANGLE_TOL 0.1
#define TRASL_TOL 0.1
#define SPEED_LIMIT_ANGLE 10

#define UPDATE_STATUS true
#define OBSTACLE_EMPIRIC true
#define OBSTACLE_DYNAMIC true
#define OBSTACLE_NEURAL false

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

    void timerPart();
    void handleWheelMotionMessage(const Data::BuddyMessage *buddy);
    void onStateUpdatedHybrid(bool isIdle);
    void onStateUpdatedHybrid();
    void onStateUpdatedNormal(bool isIdle);
    void controlRotationNewAction(const Data::Action &todo);
    void controlRotationNearToSetPoint(double distance);
    void controlRotationRobotStall();
    void controlRotationSetPointReached();
    void controlRotationNegPos();
    void controlRotationPosPos(double distance);
    void controlRotationPosNeg();
    void controlRotationNegNeg(double distance);
    void controlTraslationNewAction(const Data::Action &todo);
    void controlTraslationSetPointReached();
    void controlTraslationNearSetPoint(double distToCover, double distCovered);
    void controlTraslationTooFar();
    void controlTraslationTooBack();
    void controlTraslationStall(double distCovered, const Data::Action &todo, double distToCover);
private:
   //=== Movement Helper ===//
   /**
    * This method sets the speeds of the wheels and let the robot moves.
    * @param leftSpeed the speed of the left wheel.
    * @param rightWheel the speed of the right wheel.
    */
   void doMovement(double leftSpeed, double rightSpeed);

   /**
    * Fake method for the OpenLoop. It is not implemented.
    */
   void openLoopMovement(double angle1, double translation, double angle2);
   /**
    * This method close the loop, using the odometry sensors data.
    * @param angle1 how many degrees the robot must rotate for the initial rotation.
    * @param translation how many metres the robot must translate.
    * @param angle2 how many degrees the robot must rotate for the last rotation.
    */
   void odometryClosedLoop(double angle1, double translation, double angle2);

   //=== Control Methods ===//
   /**
    * This method is invoked to control the rotation in order to stop
    * it if the robot has reached the set point.
    * @param the action that is the rotation that must be controlled.
    */
   void controlRotation(const Data::Action &todo);
   /**
    * This method is invoked to control the translation in order to stop
    * it if the robot has reached the set point.
    * @param the action that is the translation that must be controlled.
    */
   void controlTranslation(const Data::Action &todo);

   /**
     * This routine takes the x, y, t values of the new poses
     * and checks if the values are almost the same of the old pose.
     * If yes and if there are not action in the queue, it means that
     * the robot is Idle.
     * @param comp_x the x component of the pose
     * @param comp_y the y component of the pose
     * @param comp_t the theta component of the pose
     */
   void checkIfIdle(double comp_x, double comp_y, double comp_t, double timestamp);

   /**
    * This routine takes the x, y, t values of the new poses
    * and checks if the values are almost the same of the old pose.
    * If yes and if the wheels are moving, then the robot is stalling.
    * @param comp_x the x component of the pose
    * @param comp_y the y component of the pose
    * @param comp_t the theta component of the pose
    */
  void checkIfStall(double comp_x, double comp_y, double comp_t);

  void checkAround(const Data::Pose &pose);

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

    void sigChangeStatetExplorationRCM(bool restart);

    void sigChangeStatePathPlanningRCM(bool restart);

    void sigCameraDataRCM(const Data::CameraData &camera, const Data::Pose &pose);

    void sigRestartExplorationRCM(double x, double y);

    void sigCleanBadFrontierRCM();

    void sigHandleBadFrontierRCM(const Data::Pose);

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

    void insertActionToPerform(Data::Action::ActionType type, double value);

    void onPerformActionRCM(PathPlanner::AbstractAction *action);

    void onNoFrontierAvailableRCM();

    void onHandleBadFrontierRCM(Data::Pose);

    void onPointToReachRCM(double x, double y);


    void setActionStartTimestamp(int value);
    void setControlType(int type);
    void setLastSonarData(Data::SonarData sonar);
    void stopRobot(bool saveState);
    void moveRobot(double angle1, double translation, double angle2);

private slots:

    void onRestartExploration();
    void onObstacleAvoidanceTimerExpired();
    void onPerformRandomAction();
    void recomputePath();
    void onTimeoutObstacle();
    void onTimeoutTeleoperation();

private:
    Data::Pose forwardKinematics(const Data::Pose &from, double vr, double vl, double time);
    bool poseReached(const Data::Pose &pose) const;
    void sendSonarMessage();

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
    robotAroundEnum robotAround;


    bool explorationModuleState;
    bool newAction, isSpeedChanged, isJustChanged;
    bool streamImage;
    bool isNotificationNeeded, haveReceivedWaypoint;
    bool userEnabled;
    bool slowMotion;

    uint robotId;

    int lastBatterySent;
    int sonarObstacleTimeNumber;
    int waitTime;
    int constantPoseCounter, stallCounter;
    int counterAround, countSpeedChange;
    int typeMovement;
    int refindPathCounter;
    int tryposeCounter;

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
    QTimer *obstacleAvoidanceTimer, *randomActionTimer, *teleOperationTimer;
    QTime crono;

    SLAM::SLAMModule *slam;
    PathPlanner::HybridPoseAction *goalToRecompute;

    ObstacleAvoidance *obstacleAvoidance;

    QString space;
};

#endif // ROBOTCONTROLLER_H
