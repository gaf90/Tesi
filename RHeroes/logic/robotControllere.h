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

#define TELEOPERATION_TIMEOUT_MSEC  5000
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
    int reactiveBehaviorStatus;
    int reactiveBackBehaviorStatus;

    /**
     * Constructor for the RobotController
     */
    explicit RobotController(uint id, QString initialLocation, QString initialRotation, bool isKenaf, QObject *parent = 0);
    /**
     * Destructor for the RobotController.
     */
    virtual ~RobotController();
    /**
     * This method represents the high-level primitives for the robot's movement.
     * The primitive is composed of a Rotation, a Translation and a final rotation.
     * @param angle1 how many degrees the robot must rotate for the initial rotation.
     * @param translation how many metres the robot must translate.
     * @param angle2 how many degrees the robot must rotate for the last rotation.
     */
    void moveRobot(double angle1, double translation, double angle2);

    /**
      * Return the actual state of the robot.
      * @return the state of the robot.
      */
    Data::RobotState * getActualState();

    void setSlamModule(SLAM::SLAMModule *slam);

    void setStatus(bool enable);

    void timerPart();
    void reactiveFrontCheck(const Data::SonarData &sonar);
    void reactiveBackCheck(const Data::SonarData &sonar);
    void oldFrontCheck(const Data::SonarData &sonar);
    void oldBackCheck(const Data::SonarData &sonar);
    void checkSonarStatiComplesso(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL);
    void checkSonarStatiSemplice(double distanceRightR, double distanceLeftL, double distanceLeft, double distanceFront, double distanceRight);
private:
   //=== Movement Helper ===//
   /**
    * This method sets the speeds of the wheels and let the robot moves.
    * @param leftSpeed the speed of the left wheel.
    * @param rightWheel the speed of the right wheel.
    */
   void doMovement(double leftSpeed, double rightSpeed);
   /**
    * This method stops the robot movement.
    * @param saveState if <b>true</b> stores the actual state in the pastState's stack.
    */
   void stopRobot(bool saveState);
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
   void handleSonarData(const Data::SonarData &sonar);

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
    void launchCommand(const Data::Message &command);
    /**
     * This signal is emitted to notify that the state has been updated.
     */
    void stateUpdate();

    void signalWirelessMessage(const Data::Message &message);

    //TMP signal
    void signalImage(const QImage &rawData, uint robotId);

    void movementEnd();

    void restartExplorationSignal(bool restart);

    void restartPathPlanningSignal(bool restart);

    void cameraDataSignal(const Data::CameraData &camera, const Data::Pose &pose);

    void refindPath(double x, double y);

    void userMoveCleanBadFrontiers();

    void badFrontierSignal(const Data::Pose);

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

    void updateSLAMPose(SLAM::TimedPose pose);

    void haltRobotMovement();

    void haltRobot();

    void saveActualFrontier(const Data::Pose pose);

    void insertActionToPerform(Data::Action::ActionType type, double value);

    void actionToPerform(PathPlanner::AbstractAction *action);

    void goBackNoFrontiers();

    void goStraightNoFrontiers();

    void onPathNotFound(Data::Pose);

    void waypointForKenaf(double x, double y);

private slots:

    void restartExploration();
    void obstacleAvoidanceTimerExpired();
    void performRandomAction();
    void recomputePath();
    void onTimeoutObstacle();
    void onTimeoutTeleoperation();

private:
    Data::Pose forwardKinematics(const Data::Pose &from, double vr, double vl);
    bool poseReached(const Data::Pose &pose) const;
    void sendSonarMessage();

private:
    bool isNewAction, isMovementStarted, isMovementEnded, isSpeedChanged, isSpeedJustChanged;
    QQueue<Data::Action *> *normalActionQueue;
    QQueue<PathPlanner::HybridPoseAction *> *hybridActionQueue;
    double actionStartTimestamp;
    //QStack<Data::RobotState *> *pastStates;
    Data::RobotState *pastState;
    Data::RobotState* actualState;
    uint robotId;
    bool explState;
    int lastBatterySent;
    int sonarObstacleTimeNumber;

    bool isKenaf;

    bool streamImage;
    QMutex *streamMutex;

    bool isWaypointNotificationNeeded, wayPointRecv;
    int waypointTimer;
    int constantPoseCounter, stallCounter;
    InverseKinematic *invKin;
    bool useHybridControl;

    Data::Pose pastHybridPoseToReach, previousPose;
    int counterAround, countSpeedChange;
    bool isAround, justStarted;

    QTimer *obstacleAvoidanceTimer, *randomActionTimer, *teleOperationTimer;
    QTime crono;
    bool forceStopRobotMovement, activateSonar;
    SLAM::SLAMModule *slam;
    Data::SonarData lastFrontSonarData;
    bool userEnabled;
    PathPlanner::HybridPoseAction *goalToRecompute;
    bool isTeleoperated;

    double static const maxSpeedChange;
};

#endif // ROBOTCONTROLLER_H