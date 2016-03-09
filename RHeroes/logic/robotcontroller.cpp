#include "robotcontroller.h"

#include <QDebug>
#include <QVector>
#include <QApplication>
#include <typeinfo>
#include <QStringList>
#include "shared/constants.h"
#include "shared/utilities.h"
#include "data/pose.h"
#include "data/wheelmessage.h"
#include "data/wirelessmessage.h"
#include "data/buddymessage.h"
#include "pathPlanner/pathplannerutils.h"
#include "slam/geometry/point.h"
#include <QTimer>
#include "slam/geometry/rototranslation.h"
#include "shared/random.h"
#include "shared/config.h"
#include "data/errornotificationmessage.h"
#include "inversekinematiccf.h"




using namespace Data;
using namespace PathPlanner;
using namespace SLAM::Geometry;

//======= CONSTRUCTORS =======//
RobotController::RobotController(uint id, QString initialLocation, QString initialRotation,bool aisKenaf, QObject *parent) :
    QObject(parent),
    isNewMovement(FALSE),
    wayPointCounter(0),
    normalActionQueue(new QQueue<Action *>()),
    hybridActionQueue(new QQueue<HybridPoseAction *>()),
    actionStartTimestamp(-1),
    pastState(NULL),
    actualState(new RobotState(0.0, 0.0, MAX_BATTERY)),
    robotId(id),
    previousBattery(MAX_BATTERY),
    streamImage(true),
    streamMutex(new QMutex()),
    isNotificationNeeded(false),
    haveReceivedWaypoint(false),
    waitTime(0),
    isExplorationEnabled(true),
    stallCounter(0),
    controlRobotType(NORMAL),
    previousPose(INFINITY, INFINITY, INFINITY),
    obstacleAvoidanceTimer(new QTimer()),
    teleOperationTimer(new QTimer()),
    sonarStatus(OFF),
    teleoperationStatus(OFF),
    isPathPlannerEnabled(true)

{

    //ldbg << "RobotController: Start module"<<endl;

    inverseKinematicModule = new InverseKinematicCF(WHEEL_BASE,aisKenaf);

    obstacleAvoidance = new ObstacleAvoidance(inverseKinematicModule, EMP_ANGLE_TOL);

    if (aisKenaf)
        robotType = KENAF;
    else
        robotType = P3AT;

    //Make connections
    connect(this, SIGNAL(sigRobotStateUpdated()), this, SLOT(onStateUpdated()), Qt::QueuedConnection);
    connect(obstacleAvoidanceTimer,SIGNAL(timeout()),this,SLOT(onTimeoutObstacle()), Qt::QueuedConnection);
    connect(teleOperationTimer,SIGNAL(timeout()),this,SLOT(onTimeoutTeleoperation()));

    //New
    connect(obstacleAvoidance, SIGNAL(sigChangeActionStartTimestamp(int)), this, SLOT(setActionStartTimestamp(int)), Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigUpdateSonarData(Data::SonarData)), this, SLOT(setLastSonarData(Data::SonarData)), Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigMoveRobot(double,double,double)), this, SLOT(moveRobot(double,double,double)), Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigChangeRobotControlType(int)), this, SLOT(setControlType(int)),Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigRecomputePath(Data::Pose)), this, SLOT(onRecomputePath(Data::Pose)),Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigStopRobot(bool)), this, SLOT(stopRobot(bool)),Qt::DirectConnection);
    connect(this, SIGNAL(sigChangeMovementType(int)), obstacleAvoidance, SLOT(setMovementType(int)),Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigDoMovement(double,double)), this, SLOT(sendWheelMessage(double,double)), Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigHandleTimer()), this, SLOT(onObstacleStart()), Qt::DirectConnection);
    connect(obstacleAvoidance, SIGNAL(sigFrontierReached()), this, SLOT(onFrontierReached()), Qt::DirectConnection);

    teleOperationTimer->setSingleShot(true);

    moveToThread(QApplication::instance()->thread());

    QStringList loc = initialLocation.split(",");
    QStringList rot = initialRotation.split(",");
    double x = loc[0].toDouble();
    double y = loc[1].toDouble();
    double t = rot[0].toDouble();

    previousHeading = t;

    if (Config::OBS::is_test == 1)
    {
        actualFrontier = new Pose(Config::OBS::test_frontier_y, Config::OBS::test_frontier_x, -Config::OBS::test_frontier_theta);
        oldFrontier = new Pose(actualFrontier->getX(), actualFrontier->getY(), actualFrontier->getTheta());
        actualState->setPose(Pose(Config::OBS::test_pose_y, Config::OBS::test_pose_x, Config::OBS::test_pose_theta));
    }
    else
    {
        oldFrontier = new Pose(y,x,-t);
        actualFrontier = new Pose(y,x,-t);
        actualState->setPose(Pose(y,x,-t));
    }


    pastState = new RobotState(*actualState);

}

//======= DESTRUCTORS =======//
RobotController::~RobotController()
{
    if(obstacleAvoidanceTimer->isActive())
        obstacleAvoidanceTimer->stop();
    delete obstacleAvoidanceTimer;
    if(teleOperationTimer->isActive()){
        teleOperationTimer->stop();
    }
    delete teleOperationTimer;
    delete inverseKinematicModule;
    delete actualState;
    delete pastState;
    //delete pastStates;
    while(!hybridActionQueue->isEmpty()){
        delete hybridActionQueue->dequeue();
    }
    delete hybridActionQueue;
    while(!normalActionQueue->isEmpty()){
        delete normalActionQueue->dequeue();
    }
    delete normalActionQueue;
}
//=== TIMERS HANDLING === //

void RobotController::onTimeoutTeleoperation()
{
    //ldbg << "RobotController: Teleoperation Timeout expired. Is active exploration module? "<<explorationModuleState<<". Is active user? " <<userEnabled<<endl;

    if (isExplorationEnabled && isPathPlannerEnabled)
    {
        sonarStatus = ON;
        teleoperationStatus = OFF;
        obstacleAvoidanceTimer->stop();
        ldbg << "Robot controller: Restart exploration after teleoperation timeout"<<endl;
        onRestartExploration();
    }
}


//======= SENSORS HANDLING =======//


void RobotController::onSensorData(const Message &data)
{
    if(typeid(data) == typeid(const CameraData &))
    {
        const CameraData &camera = (const CameraData &)data;
        //ldbg << "Robot Controller: Received camera data at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<endl;
        handleCameraData(camera);
    }
    else if(typeid(data) == typeid(const StateData &))
    {
        // ldbg << "Robot Controller: Received state data at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<endl;
        const StateData &state = (const StateData &)data;
        handleStateData(state);
    }
    else if(typeid(data) == typeid(const SonarData &) )
    {
        // ldbg << "Robot Controller: Received sonar data at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<endl;
        const SonarData &sonar = (const SonarData &)data;

        const Data::Action *actualAction = NULL;
        if (!normalActionQueue->isEmpty())
            actualAction = normalActionQueue->head();

        obstacleAvoidance->handleObstacle(sonar, pastState,actualState,actualAction,actualFrontier);
    }

    else if(typeid(data) == typeid(const LaserData &) )
    {
        //ldbg << "Robot Controller: Received laser data at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<endl;
        Data::LaserData &laser = (LaserData &)data;
        obstacleAvoidance->setLaser(laser);

    }

    else if(typeid(data) == typeid(const WirelessMessage &))
    {
        // ldbg << "Robot Controller: Received wireless data at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<endl;
        handleWirelessData(data);
    }
}

void RobotController::handleCameraData(const CameraData &camera)
{
    if(!streamImage)
    {
        ldbg << "Robot Controller - camera: Not streaming"<<endl;
        return;
    }

    BuddyMessage buddyMessage(robotNameFromIndex(robotId),
                              robotNameFromIndex(BASE_STATION_ID), &camera);
    WirelessMessage wireless(&buddyMessage);

    //Send camera data message to Wireless Driver
    emit sigWirelessMessageRCM(wireless);

    //Send camera data and pose to Victim Detection
    emit sigCameraDataRCM(camera, actualState->getPose());
}

void RobotController::handleStateData(const Data::StateData &state)
{
    double actualBattery = state.getBattery();
    actualState->setBattery(actualBattery);
    if(actualBattery == previousBattery - BATTERY_ONE_PERCENT)
    {
        previousBattery = actualBattery;
        InfoMessage infoMessage(BATTERY_STATUS, QString::number(previousBattery));
        BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID), &infoMessage );
        WirelessMessage wirelessMessage(&buddy);

        //Send battery message to Wireless Driver
        emit sigWirelessMessageRCM(wirelessMessage);

    }
}

void RobotController::handleWheelMotionMessage(const BuddyMessage *buddy)
{
    const WheelMessage *wheelMessage = buddy->getWheelMessage();
    ldbg <<"Robot controller - wheel motion: Stop robot"<<endl;
    stopRobot(true);

    sonarStatus = OFF;
    teleoperationStatus = ON;
    double leftSpeed = wheelMessage->getLeftWheelSpeed();
    double rightSpeed = wheelMessage->getRightWheelSpeed();
    if(leftSpeed == 0 && rightSpeed == 0)
    {
        if(teleOperationTimer->isActive())
            teleOperationTimer->stop();

        ldbg << "Robot controller - wheel motion: Restart teleoperation timer."<<endl;
        teleOperationTimer->start(TIMEOUT_MSEC);
    }
    ldbg <<"Robot controller - wheel motion: Deactivate exploration and path planner. Clean bad frontier."<<endl;

    isExplorationEnabled = false;
    isPathPlannerEnabled = false;
    emit sigChangeStateExplorationRCM(false);
    emit sigChangeStatePathPlanningRCM(false);

    emit sigCleanBadFrontierRCM();

    ldbg <<"Robot controller - wheel motion: Do movement ("<< leftSpeed <<"," << rightSpeed <<","<<endl;

    sendWheelMessage(leftSpeed, rightSpeed);
}

void RobotController::sendWheelMessage(double leftSpeed, double rightSpeed)
{
    actualState->setLeftSpeed(leftSpeed);
    actualState->setRightSpeed(rightSpeed);
    WheelMessage command(leftSpeed, rightSpeed);
    emit sigDriverMessageRobot(command);
}

void RobotController::handleWirelessData(const Message &data)
{
    const WirelessMessage &message = (const WirelessMessage &)data;
    if(message.getCommand() == WirelessMessage::MessageExchange){
        const BuddyMessage *buddy = message.getBuddyMessage();
        if(buddy->getContent() == BuddyMessage::WheelMotion)
            handleWheelMotionMessage(buddy);
    }
}

void RobotController::handleWaypoint(const WaypointCommand * waypoint)
{
    actualWaypoint = new WaypointCommand(waypoint->getWaypoint(), false, 0.0);
    ldbg <<"Robot Controller: Received waypoint. Stop robot"<<endl;
    stopRobot(true);

    emit sigCleanBadFrontierRCM();

    sonarStatus = ON;
    controlRobotType = NORMAL;
    haveReceivedWaypoint = true;
    isNotificationNeeded = waypoint->isNotificationNeeded();
    waitTime = waypoint->getTimer();
    Pose pointToReach = waypoint->getWaypoint();
    double rotation = fromRadiantToDegree(computeRotationFromPoses(actualState->getPose(), pointToReach));

    double dx2 = pow(pointToReach.getX()- actualState->getPose().getX(), 2);
    double dy2 = pow(pointToReach.getY()- actualState->getPose().getY(), 2);
    double trasl = sqrt(dy2+dx2);

    if (rotation>0 && rotation<EMP_ANGLE_TOL)
        rotation+=EMP_ANGLE_TOL;
    else if (rotation<0 && rotation>-EMP_ANGLE_TOL)
        rotation-=EMP_ANGLE_TOL;

    moveRobot(rotation, trasl, 0.0);
}


//======= PATH PLANNER =======//

void RobotController::onPerformActionRCM(PathPlanner::AbstractAction *action)
{
    sonarStatus = ON;
    if (obstacleAvoidance->empiricBehaviorStatus>0 || obstacleAvoidance->dwaBehaviorStatus>0 || obstacleAvoidance->neuralBehaviorStatus>0)
    {
        ldbg << "Robot Controller: Ignore perform action" << endl;
        return;
    }

    if(typeid(*action) == typeid(Data::Action))
    {
        Data::Action *act = (Data::Action *)action;
        if(act->getType() == Action::Rotation)
        {
            double rotation = act->getValue();

            if (rotation>0 && rotation<EMP_ANGLE_TOL)
                rotation+=EMP_ANGLE_TOL;
            else if (rotation<0 && rotation>-EMP_ANGLE_TOL)
                rotation-=EMP_ANGLE_TOL;

            moveRobot(rotation, 0.0, 0.0);
        }
        else
        {
            double translation = act->getValue();

            if (translation>0 && translation<TRASL_TOL)
                translation+=TRASL_TOL;
            else if (translation<0 && translation>-TRASL_TOL)
                translation-=TRASL_TOL;

            moveRobot(0.0,translation, 0.0);
        }

        controlRobotType = NORMAL;
    }
    else if(typeid(*action) == typeid(HybridPoseAction))
    {
        HybridPoseAction *act = (HybridPoseAction *)action;
        HybridPoseAction *toEnqueue = new HybridPoseAction(act->getValue());
        hybridActionQueue->enqueue(toEnqueue);
        controlRobotType = HYBRID;
    }
}

//======= SLAM =======//
void RobotController::onNewRobotPose(SLAM::TimedPose pose)
{
    double distance = actualState->getPose().getDistance(pose);
    double rotation = wrapDeg(fromRadiantToDegree(angularDistance(actualState->getPose().getTheta(), pose.getTheta())));
    //ldbg << "Robot Controller - SLAM: Pose is " << pose <<" at "<<QTime::currentTime().toString("hh:mm:ss:zzz");
    //ldbg << ". D = "<<distance<<", R = "<<rotation <<endl;
    if(distance<TRASL_TOL && abs(rotation) <EMP_ANGLE_TOL)
    {
        //ldbg << "Robot Controller: Slam pose received: " << pose <<" at "<<QTime::currentTime().toString("hh:mm:ss:zzz");
        //ldbg << ". D = "<<distance<<", R = "<<rotation <<endl;
        if (normalActionQueue->isEmpty())
        {
            stallCounter++;
            if(stallCounter > STALL_NUMBER)
            {
                // ldbg <<"Robot Controller: Robot is doing nothing for "<< stallCounter <<" times!"<<endl;
                //ldbg <<"Robot controller: Robot is stall! Restart exploration"<<endl;
                stallCounter = 0;
                actualState->setStall(true);            }
        }
        else
        {
            stallCounter = 0;
            actualState->setStall(false);
        }
    }
    else
    {
        //ldbg <<"Robot controller: Robot moved from "<<actualState->getPose() << " to "<<pose <<" at "<<QTime::currentTime().toString("hh:mm:ss:zzz") <<".Update state data"<<endl;
        previousPose = actualState->getPose();
        actualState->setPose(pose);
        actualState->setTimestamp(pose.getTimestamp());
        if(pastState == NULL){
            pastState = new RobotState(actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());
            pastState->setPose(pose);
            pastState->setTimestamp(pose.getTimestamp());
        }
    }

    emit sigRobotStateUpdated();
}


//======= ROBOT MOTION =======//

void RobotController::moveRobot(double angle1, double translation, double angle2)
{
    //I build and enqueue the first action: ROTATION
    if(angle1 != 0.0){
        ldbg << "Robot Controller: I have received a rotation of "<< angle1<<endl;
        double radiant1 = fromDegreeToRadiants(angle1);
        Action* act1 = new Action();
        act1->setType(Action::Rotation);
        act1->setValue(radiant1);
        normalActionQueue->enqueue(act1);
        isNewMovement = TRUE;
    }
    //I build and enqueue the second action: TRANSLATION
    if(translation != 0.0){
        ldbg << "Robot Controller: I have received a translation of "<< translation<<endl;
        Action* act2 = new Action();
        act2->setType(Action::Translation);
        act2->setValue(translation);
        normalActionQueue->enqueue(act2);
        isNewMovement = TRUE;
    }
    //I build and enqueue the third action: ROTATION
    if(angle2 != 0.0){
        ldbg << "Robot Controller: I have received a rotation of "<< angle2<<endl;
        double radiant3 = fromDegreeToRadiants(angle2);
        Action* act3 = new Action();
        act3->setType(Action::Rotation);
        act3->setValue(radiant3);
        normalActionQueue->enqueue(act3);
        isNewMovement = TRUE;    }
}

void RobotController::stopRobot(bool saveState)
{
    ldbg << "RobotController: stop robot."<<endl;

    sendWheelMessage(0,0);
    if(saveState){
        saveRobotState();
    }

    while(!normalActionQueue->isEmpty()){
        delete normalActionQueue->dequeue();
    }

    while(!hybridActionQueue->isEmpty())
        delete hybridActionQueue->dequeue();

}

void RobotController::saveRobotState()
{
    Pose* newPose = new Pose(actualState->getPose().getX(), actualState->getPose().getY(),
                             actualState->getPose().getTheta());
    RobotState* newState = new RobotState(*newPose, actualState->getTimestamp(),
                                          actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());

    delete pastState;
    pastState = newState;
}

void RobotController::onStopRobotForPlanning()
{

    if (obstacleAvoidance->empiricBehaviorStatus>0 || obstacleAvoidance->dwaBehaviorStatus>0 || obstacleAvoidance->neuralBehaviorStatus>0)
    {
        ldbg << "Robot Controller: Ignore path planner."<<endl;
    }
    else
    {
        stopRobot(true);
        isExplorationEnabled = false;
        emit sigChangeStateExplorationRCM(false);
    }
}


void RobotController::onRobotNotReachable()
{
    if (obstacleAvoidance->empiricBehaviorStatus>0 || obstacleAvoidance->dwaBehaviorStatus>0 || obstacleAvoidance->neuralBehaviorStatus>0)
    {
        ldbg << "Robot Controller: Ignore wss. Doing last movement."<<endl;
    }
    else
    {
        ldbg << "Robot Controller:  Path planner stops the robot. No restart exploration." << endl;
        stopRobot(true);
    }
}

bool RobotController::poseReached(const Pose &pose)
{
    const Pose actualPose = actualState->getPose();
    const Point actualFrontierPoint(pose.x(), pose.y());
    const Point robotLocation(actualPose.x(), actualPose.y());

    const bool directCheck = robotLocation.distance(actualFrontierPoint) <= INVERSE_KIN_TRASL_TOL &&
            abs(wrapRad(actualPose.theta() - pose.theta())) <= INVERSE_KIN_ANG_TOL;

    if(std::isinf(previousPose.theta()))
    {
        /* There is no previous pose yet */
        return directCheck;
    }
    else
    {
        const LineSegment path(previousPose.x(), previousPose.y(), actualPose.x(), actualPose.y());
        double theta1 = actualPose.theta(), theta2 = actualPose.theta();
        bool angularOvershoot;
        if(theta2 < theta1)
            exchange(theta1, theta2);
        if(theta1 + M_PI < theta2)
        {
            angularOvershoot = pose.theta() >= theta2 && pose.theta() <= theta1;
        }
        else
        {
            angularOvershoot = pose.theta() >= theta1 && pose.theta() <= theta2;
        }
        return (path.intersectsCircle(actualFrontierPoint, INVERSE_KIN_TRASL_TOL) &&
                angularOvershoot) || directCheck;
    }
}

//======= STATES UPDATE PROCESSING =======//

void RobotController::onStateUpdated()
{
    //ldbg << "Robot Controller - onStateUpdate: start";

    bool isIdle = actualState->isIdle();

    if(controlRobotType == HYBRID && !hybridActionQueue->isEmpty())
        onStateUpdatedHybrid(isIdle);
    else if (controlRobotType == NORMAL && !normalActionQueue->isEmpty())
        onStateUpdatedNormal();
    else
    {
        if(isIdle && teleoperationStatus == OFF && !isExplorationEnabled)
        {
            isNewMovement = FALSE;
            ldbg <<"Robot controller: Robot is idle, teleoperation is off. Stop robot and restart "<<endl;
            stopRobot(true);
            obstacleAvoidance->empiricBehaviorStatus = 0;
            obstacleAvoidance->dwaBehaviorStatus = 0;
            obstacleAvoidance->neuralBehaviorStatus = 0;
            isExplorationEnabled = true;
            emit sigChangeStateExplorationRCM(true);
        }
        if (haveReceivedWaypoint && (obstacleAvoidance->empiricBehaviorStatus  == 0 && obstacleAvoidance->neuralBehaviorStatus==0 && obstacleAvoidance->dwaBehaviorStatus == 0))
            notifyAfterWaypoint();
    }
}

void RobotController::notifyAfterWaypoint()
{
    //ldbg << "Robot Controller - notifying after waypoint!" << endl;
    //ldbg << haveReceivedWaypoint<<endl;
    if(haveReceivedWaypoint)
    {
        if(isExplorationEnabled && isPathPlannerEnabled)
        {
            if(waitTime > 0){
                QTimer::singleShot(waitTime*MILLIS, this, SLOT(onRestartExploration()));
            } else {
                ldbg << "Robot controller: Restart exploration after waypoint"<<endl;
                onRestartExploration();
            }
            haveReceivedWaypoint = false;
            delete actualWaypoint;
        }
        else
        {
            //Disable sonar
            //ldbg<<"Robot Controller - Disable sonar"<<endl;
            sonarStatus = OFF;
        }
    }


    if(isNotificationNeeded)
    {
        isNotificationNeeded = false;
        ldbg <<"Robot Controller: robot finish movement. Stop and notify"<<endl;
        stopRobot(true);
        //Create the information
        QString str(robotNameFromIndex(robotId));
        str = str.append(" has finished its movement");
        InfoMessage info(MOVEMENT_END, str);
        BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID), &info);
        WirelessMessage wm(&buddy);
        emit sigWirelessMessageRCM(wm);
        //ldbg << "Robot Controller - message waypoint movement finished" << endl;

    }
}


void RobotController::onStateUpdatedHybrid(bool isIdle)
{
    HybridPoseAction *act = hybridActionQueue->head();

    if(poseReached(act->getValue()) || isIdle)
    {
        actionStartTimestamp = actualState->getTimestamp();
        act = hybridActionQueue->dequeue();
        delete act;
        saveRobotState();

    }

    if(hybridActionQueue->isEmpty())
    {
        ldbg << "Robot Controller - onStateUpdated: The queue is empty!. Stop robot" << endl;
        actionStartTimestamp = -1;
        stopRobot(false);
    }
    else
    {
        if(actionStartTimestamp < 0)
            actionStartTimestamp = actualState->getTimestamp();

        act = hybridActionQueue->head();

        const Data::Pose displacedPose(SLAM::Geometry::Rototranslation(actualState->getPose(), act->getValue()).vectorForm());
        pastHybridPoseToReach = displacedPose;

        double timeLeft = actionStartTimestamp + DELTA_T - actualState->getTimestamp();
        if(timeLeft < DELTA_T/5)
            timeLeft = DELTA_T/5;

        WheelSpeeds ws = inverseKinematicModule->computeSpeeds(displacedPose, timeLeft);

        if(controlRobotType == HYBRID)
            sendWheelMessage(ws.getLeftSpeed(), ws.getRightSpeed());
    }

    previousPose = actualState->getPose();
}

void RobotController::onStateUpdatedNormal()
{
    Action* actualAction = normalActionQueue->head();
    switch(actualAction->getType())
    {
    //==== ROTATION CASE ====//
    case Action::Rotation:
        controlRotation(*actualAction);
        break;
        //==== TRANSLATION CASE ====//
    case Action::Translation:
        controlTranslation(*actualAction);
        break;
    default:

        ldbg<<"Robot Controller - Error"<<endl;
        stopRobot(true);
        break;
    }
}



void RobotController::controlRotation(const Action &rotation)
{
    if(actualState->isStall())
    {
        ldbg<<"Robot Controller - Control rotation: Stall."<<endl;
        handleRobotStall();
    }
    else
    {
        double deltaHeading = rotation.getValue();
        double actualHeading = actualState->getPose().getTheta();
        double pastHeading = pastState->getPose().getTheta();

        if(isNewMovement)
            controlStartRotation(deltaHeading);
        else if (actualHeading!=previousHeading)
        {
            previousHeading = actualHeading;
            double finalHeading = wrapRad(pastHeading + deltaHeading);
            double lastHeading = fromRadiantToDegree(angularDistance(actualHeading, finalHeading));
            ldbg<<" "<<endl;
            ldbg <<"Robot Controller - Control Rotation: DH = "<<fromRadiantToDegree(deltaHeading) <<
                   ", PH = " << fromRadiantToDegree(pastHeading) <<
                   ", AH = "<< fromRadiantToDegree(actualHeading) <<
                   ", FH = "<< fromRadiantToDegree(finalHeading)<<
                   ", LH = "<< lastHeading <<endl;
            if(fabs(lastHeading) <= EMP_ANGLE_TOL)
            {
                controlEndRotation();
            }
            else if (deltaHeading>0)
            {
                controlLeftRotation(lastHeading);
            }
            else if(deltaHeading<0)
            {
                controlRightRotation(lastHeading);
            }
        }

    }
}

void RobotController::controlStartRotation(double rotation)
{
    if(rotation> 0)
    {
        //ldbg << "Robot Controller - ControlRotation: I rotate counter-clockwise!"<< endl;
        sendWheelMessage(-WHEEL_SPEED, WHEEL_SPEED);
    }
    else
    {
        //ldbg << "Robot Controller - ControlRotation: I rotate clockwise!"<< endl;
        sendWheelMessage(WHEEL_SPEED, -WHEEL_SPEED);
    }
    isNewMovement = FALSE;
}

void RobotController::handleRobotStall()
{
    stopRobot(false);
    isNewMovement = TRUE;
    ldbg << "Robot Controller: Stop robot. Restart exploration after stall"<<endl;
    onRestartExploration();
}

void RobotController::controlEndRotation()
{
    ldbg << "Robot Controller - Control Rotation: Set point reached!" << endl;

    sendWheelMessage(0,0);
    emit sigChangeMovementType(S);

    delete normalActionQueue->dequeue();

    //Push the reached state into the states history
    saveRobotState();

    isNewMovement = TRUE;
    onStateUpdated();
}

void RobotController::controlLeftRotation(double lastHeading)
{
    if(lastHeading > 0)
    {
        ldbg<<"Control Rotation: continue rotate left"<<endl;
        sendWheelMessage(-WHEEL_SPEED, WHEEL_SPEED);
    }
    else
    {
        if (actualState->getLeftSpeed()<0)
        {
            ldbg<<"Control Rotation: invert rotation(left to right)"<<endl;
            sendWheelMessage(WHEEL_SPEED, -WHEEL_SPEED);
        }
        else
        {
            ldbg <<"Control Rotation: Already rotate right. No invert"<<endl;
        }
    }
}

void RobotController::controlRightRotation(double lastHeading)
{
    if(lastHeading < 0)
    {
        ldbg<<"Control Rotation: continue rotate right"<<endl;
        sendWheelMessage(WHEEL_SPEED, -WHEEL_SPEED);
    }
    else
    {
        if (actualState->getLeftSpeed()>0)
        {
            ldbg<<"Control Rotation: invert rotation(right to left)"<<endl;
            sendWheelMessage(-WHEEL_SPEED, WHEEL_SPEED);
        }
        else
        {
            ldbg <<"Control Rotation: Already rotate left. No invert"<<endl;
        }
    }
}

void RobotController::controlTranslation(const Action &traslation)
{
    if(isNewMovement)
        controlStartTranslation(traslation);
    else if (previousX != actualState->getPose().getX() && previousY != actualState->getPose().getY())
    {
        previousX = actualState->getPose().getX();
        previousY = actualState->getPose().getY();
        //It is already moving forward
        double distToCover = traslation.getValue();
        double distCoveredSquare = pow(actualState->getPose().getX()- pastState->getPose().getX(), 2) +
                pow(actualState->getPose().getY()-pastState->getPose().getY(), 2);
        double distCovered = 0;
        if(distToCover > 0)
            distCovered = sqrt(distCoveredSquare);
        else
            distCovered = -sqrt(distCoveredSquare);

        double distanceDelta = distToCover - distCovered;



        if(abs(distanceDelta)<= TRASL_TOL)
        {
            ldbg<<"RobotController - Control Traslation: I covered " << distCovered <<". Set point reached!"<<endl;
            controlEndTranslation();
        }
        else if (distanceDelta  < -TRASL_TOL)
        {
            controlTraslationTooFar();
        }
        else if(distanceDelta > TRASL_TOL)
        {
            controlTraslationTooBack();
        }
    }
}

void RobotController::controlStartTranslation(const Action &traslation)
{
    if(traslation.getValue()>0)
    {
        ldbg << "Robot Controller - controlTranslation: Go straight of: "<< traslation.getValue()<<endl;
        sendWheelMessage(MED_SPEED, MED_SPEED);
    }
    else
    {
        ldbg << "Robot Controller - controlTranslation: Go back of: "<< traslation.getValue()<<endl;
        sendWheelMessage(-MED_SPEED, -MED_SPEED);
    }

    isNewMovement = FALSE;
}

void RobotController::controlEndTranslation()
{

    sendWheelMessage(0,0);
    emit sigChangeMovementType(S);

    delete normalActionQueue->dequeue();

    //Push the reached state into the states history
    saveRobotState();

    isNewMovement = TRUE;
    onStateUpdated();
}

void RobotController::controlTraslationTooFar()
{
    double leftSpeed=actualState->getLeftSpeed();
    double rightSpeed=actualState->getRightSpeed();

    if(((fabs(leftSpeed))>MED_SPEED))
        leftSpeed = leftSpeed * SPEED_DECR_FACTOR;
    else
        leftSpeed = MED_SPEED;

    if(fabs(rightSpeed)>MED_SPEED)
        rightSpeed = rightSpeed * SPEED_DECR_FACTOR;
    else
        rightSpeed = MED_SPEED;

    sendWheelMessage(-leftSpeed, -rightSpeed);
}

void RobotController::controlTraslationTooBack()
{
    double leftSpeed=actualState->getLeftSpeed();
    double rightSpeed=actualState->getRightSpeed();

    if(abs(leftSpeed)>MED_SPEED)
        leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
    else
        leftSpeed=MED_SPEED;
    if(abs(rightSpeed)>MIN_SPEED)
        rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
    else
        rightSpeed=MED_SPEED;

    sendWheelMessage(leftSpeed, rightSpeed);
}

void RobotController::onRecomputePath(const Data::Pose &actualFrontier)
{
    ldbg<<"Robot controller: Recompute path. Move 0.3"<<endl;
    moveRobot(0,0.3,0);
    emit sigRecomputePathRCM(actualFrontier);
    emit sigChangeStatePathPlanningRCM(true);
}

void RobotController::onRestartExploration()
{
    if(isExplorationEnabled && isPathPlannerEnabled)
    {
        ldbg <<"Robot Controller: Restart exploration"<<endl;
        emit sigChangeStateExplorationRCM(true);
        emit sigChangeStatePathPlanningRCM(true);
    }
}

void RobotController::onObstacleStart()
{
    if (!obstacleAvoidanceTimer->isActive())
    {
        frontierTime.restart();
        obstacleAvoidanceTimer->start(TIMEOUT_MSEC);
        ldbg << "Robot controller: Obstacle start."<<endl;
    }
}

void RobotController::onTimeoutObstacle()
{
    ldbg << "Robot controller: Restart exploration after "<<frontierTime.elapsed()<<endl;
    obstacleAvoidanceTimer->stop();
    if (!hybridActionQueue->isEmpty()){
        Pose goal = hybridActionQueue->head()->getValue();
        ldbg << "Robot Controller - I cannot reach pose "<<goal.getX() << ","<<goal.getY()<<endl;
        normalActionQueue->clear();
        hybridActionQueue->clear();
        emit sigHandleBadFrontierRCM(goal);
    }
    onRestartExploration();
}

void RobotController::onFrontierReached()
{
    ldbg << "Robot controller: Reached frontier after "<<frontierTime.elapsed()<<endl;
    obstacleAvoidanceTimer->stop();
}

void RobotController::onHandleBadFrontierRCM(Data::Pose pose)
{
    if (actualFrontier->getX() == pose.getX() && actualFrontier->getY() == pose.getY())
    {
        emit sigHandleBadFrontierRCM(pose);
    }

    controlRobotType = NORMAL;
    sonarStatus = ON;
    double rotation = fromRadiantToDegree(computeRotationFromPoses(actualState->getPose(), pose));
    if (rotation>0 && rotation<EMP_ANGLE_TOL)
        rotation+=EMP_ANGLE_TOL;
    else if (rotation<0 && rotation>-EMP_ANGLE_TOL)
        rotation-=EMP_ANGLE_TOL;

    moveRobot(rotation,1,0);
}

void RobotController::onNoFrontierAvailableRCM()
{
    emit sigCleanBadFrontierRCM();
    controlRobotType = NORMAL;;
    moveRobot(0,EMP_GO_STRAIGHT,0);
}

void RobotController::setSlamModule(SLAM::SLAMModule *slam)
{
    this->slam = slam;
    obstacleAvoidance->setSlamModule(slam);
}

void RobotController::setStatus(bool status)
{
    isPathPlannerEnabled = status;
    isExplorationEnabled = status;
    if(status){
        //ldbg << "Robot Controller - setStatus: enabling things..." << endl;
        emit sigChangeStateExplorationRCM(true);
        emit sigChangeStatePathPlanningRCM(true);
    } else {
        //ldbg << "Robot Controller - setStatus: disabling things..." << endl;
        emit sigChangeStateExplorationRCM(false);
        emit sigChangeStatePathPlanningRCM(false);
        onStopRobotForPlanning();
    }
}

void RobotController::onFrontierToReachRCM(const Pose frontier)
{
    if (frontier.getX() != oldFrontier->getX() && frontier.getY() != oldFrontier->getY())
    {
        delete actualFrontier;
        actualFrontier = new Pose(frontier.getX(),frontier.getY(), frontier.getTheta());
        delete oldFrontier;
        oldFrontier = new Pose(actualFrontier->getX(),actualFrontier->getY(), actualFrontier->getTheta());

    }
}

void RobotController::onPointToReachRCM(double x, double y)
{
    WaypointCommand wpCmd(Pose(x, y, 0.0), false, 0.0);
    wayPointCounter = 0;
    handleWaypoint(&wpCmd);
}

void RobotController::setActionStartTimestamp(int value)
{
    this->actionStartTimestamp = value;
}

void RobotController::setLastSonarData(Data::SonarData sonar)
{
    this->lastFrontSonarData = sonar;
}

void RobotController::setControlType(int type)
{
    this->controlRobotType = (controlTypeEnum)type;
}

RobotState * RobotController::getActualState()
{
    return actualState;
}

void RobotController::updatePose(const Pose &pose)
{
    previousPose = actualState->getPose();
    actualState->setPose(pose);
}

void RobotController::setStreamImageFlag(bool stream)
{
    streamMutex->lock();
    streamImage = stream;
    streamMutex->unlock();
}





