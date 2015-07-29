#include "robotcontroller.h"

#include <QDebug>
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
#include "time.h";

#define THRESHOLD 0.23
#define RUOTASINISTRA_MIN 10
#define RUOTASINISTRA_MED 20
#define RUOTASINISTRA_MAX 40
#define RUOTADESTRA_MIN -10
#define RUOTADESTRA_MED -20
#define RUOTADESTRA_MAX -40
#define VAI_AVANTI 0.5
#define VAI_INDIETRO -0.15

#define UPDATE_STATUS true
#define YES true
#define NO false

const double RobotController::maxSpeedChange = 0.3;
QString space = "";
enum reactiveBehaviorEnum {DEACTIVATED,FIRSTTIME,EXEC};
enum typeMovementEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
enum movementStateEnum {FRONT,RIGHT,LEFT,BACK};
movementStateEnum actualMovement ;
int typeMovement = S;
bool slowMotion = false;
bool setPointReachedTranslation = true;
bool setPointBackTranslation = true;
bool setPointReachedRotation = true;
bool firstTime = true;
Data::WaypointCommand *actualWaypoint;
Data::Pose *actualFrontier;
Data::Pose *oldFrontier;
int refindPathCounter;
int tryposeCounter = 0;
double oldx = 0.0, oldy = 0.0;

using namespace Data;
using namespace PathPlanner;
using namespace SLAM::Geometry;

//======= CONSTRUCTORS =======//
RobotController::RobotController(uint id, QString initialLocation, QString initialRotation,bool aisKenaf, QObject *parent) :
    QObject(parent),
    isNewAction(FALSE),
    wayPointCounter(0),
    isMovementStarted(FALSE),
    isMovementEnded(FALSE),
    isSpeedChanged(FALSE),
    isSpeedJustChanged(FALSE),
    normalActionQueue(new QQueue<Action *>()),
    hybridActionQueue(new QQueue<HybridPoseAction *>()),
    actionStartTimestamp(-1),
    pastState(NULL),
    actualState(new RobotState(0.0, 0.0, MAX_BATTERY)),
    robotId(id),
    lastBatterySent(MAX_BATTERY),
    streamImage(true),
    streamMutex(new QMutex()),
    isWaypointNotificationNeeded(false),
    wayPointRecv(false),
    waypointTimer(0),
    explState(true),        //PRM
    reactiveBehaviorStatus(DEACTIVATED),
    reactiveBackBehaviorStatus(DEACTIVATED),
    constantPoseCounter(0),
    stallCounter(0),
    sonarObstacleTimeNumber(0),
    useHybridControl(false),
    previousPose(INFINITY, INFINITY, INFINITY),
    countSpeedChange(0),
    justStarted(false),
    obstacleAvoidanceTimer(new QTimer()),
    randomActionTimer(new QTimer()),
    teleOperationTimer(new QTimer()),
    forceStopRobotMovement(false),
    activateSonar(false),
    userEnabled(true),      //PRM
    goalToRecompute(NULL),
    isTeleoperated(false),
    isKenaf(aisKenaf)
{
    invKin = new InverseKinematicCF(WHEEL_BASE,isKenaf);
    //Make connections
    connect(this, SIGNAL(stateUpdate()), this, SLOT(onStateUpdated()), Qt::QueuedConnection);
    connect(obstacleAvoidanceTimer,SIGNAL(timeout()),this,SLOT(onTimeoutObstacle()), Qt::QueuedConnection);
    connect(teleOperationTimer,SIGNAL(timeout()),this,SLOT(onTimeoutTeleoperation()));
    teleOperationTimer->setSingleShot(true);

    refindPathCounter = 0;
    moveToThread(QApplication::instance()->thread());

    QStringList loc = initialLocation.split(",");
    QStringList rot = initialRotation.split(",");
    double x = loc[0].toDouble();
    double y = loc[1].toDouble();
    double t = rot[0].toDouble();

    oldFrontier = new Pose(x,y,t);
    actualFrontier = new Pose(x,y,t);

    actualState->setPose(Pose(x, y, t));
    pastState = new RobotState(*actualState);
    obstacleAvoidanceTimer->setSingleShot(true);
    randomActionTimer->setSingleShot(true);
}

//======= DESTRUCTORS =======//
RobotController::~RobotController()
{
    if(goalToRecompute != NULL)
        delete goalToRecompute;
    if(randomActionTimer->isActive())
        randomActionTimer->stop();
    delete randomActionTimer;
    if(obstacleAvoidanceTimer->isActive())
        obstacleAvoidanceTimer->stop();
    delete obstacleAvoidanceTimer;
    if(teleOperationTimer->isActive()){
        teleOperationTimer->stop();
    }
    delete teleOperationTimer;
    delete invKin;
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
void RobotController::onTimeoutTeleoperation()
{
    if (explState && userEnabled)
    {
        activateSonar = true;
        isTeleoperated = false;
        randomActionTimer->stop();
        obstacleAvoidanceTimer->stop();
        qDebug()<<"Timeout.Esploro"<<endl;
        restartExploration();
    }
}

//======= SENSORS HANDLING =======//
void RobotController::handleWirelessData(const Message &data)
{
    const WirelessMessage &message = (const WirelessMessage &)data;
    if(message.getCommand() == WirelessMessage::MessageExchange){
        const BuddyMessage *buddy = message.getBuddyMessage();
        if(buddy->getContent() == BuddyMessage::WheelMotion){
            const WheelMessage *wheelMessage = buddy->getWheelMessage();
            ////ldbg<<"Robot Controller: Arrived wheel motion message. Speed  "<< wheelMessage->getLeftWheelSpeed()<<" , "<<wheelMessage->getRightWheelSpeed()<<endl;
            ////ldbg << "Robot Controller: Stop robot, activate teleoperation, deactivate sonar."<<endl;

            stopRobot(UPDATE_STATUS);
            activateSonar = NO;
            isTeleoperated = YES;

            if(almostEqual(wheelMessage->getLeftWheelSpeed(),0,0.01)&&almostEqual(wheelMessage->getRightWheelSpeed(),0,0.01)){
                if(teleOperationTimer->isActive()){
                    teleOperationTimer->stop();
                }
                ldbg << "handleWirelessData. restart timers"<<endl;
                crono.start();
                teleOperationTimer->start(TELEOPERATION_TIMEOUT_MSEC);
            }
            emit restartExplorationSignal(NO);
            emit restartPathPlanningSignal(NO);
            emit userMoveCleanBadFrontiers();
            //                //ldbg<<"We are in teleoperation. Stopping robot and restart exploration, path planning and cleaning bad frontiers."<<endl;

 ////ldbg << "Robot Controller: Apply wheel speed." << endl;
       doMovement(wheelMessage->getLeftWheelSpeed(), wheelMessage->getRightWheelSpeed());
        }
    }
}
void RobotController::onSensorData(const Message &data)
{
    if(typeid(data) == typeid(const CameraData &)){
        const CameraData &camera = (const CameraData &)data;
        ////ldbg <<"Robot Controller: We have received camera data"<<endl;
        handleCameraData(camera);
    } else if(typeid(data) == typeid(const StateData &)){
        const StateData &state = (const StateData &)data;
        ////ldbg <<"Robot Controller: We have received state data"<<endl;
        handleStateData(state);
    } else if(typeid(data) == typeid(const SonarData &)){
        const SonarData &sonar = (const SonarData &)data;
        // //ldbg <<"Robot Controller: We have received sonar data"<<endl;
        handleSonarData(sonar);
    } else if(typeid(data) == typeid(const WirelessMessage &)){
        ////ldbg <<"Robot Controller: We have received wireless data"<<endl;
        handleWirelessData(data);
    }

}

void RobotController::handleStateData(const Data::StateData &state)
{
    actualState->setBattery(state.getBattery());
    if(state.getBattery() == lastBatterySent - BATTERY_ONE_PERCENT){
        lastBatterySent = state.getBattery();
        //Build the battery status message
        InfoMessage infoMessage(BATTERY_STATUS, QString::number(lastBatterySent));
        //        //ldbg <<space << "Sending a battery message with value "<<lastBatterySent << endl;
        //Build the buddy message
        BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID), &infoMessage );
        //Build the wireless message
        WirelessMessage wirelessMessage(&buddy);
        //Send the wireless
        emit signalWirelessMessage(wirelessMessage);

    }
}

void RobotController::checkIfIdle(double comp_x, double comp_y, double comp_t, double timestamp)
{
    const Pose pose(comp_x, comp_y, comp_t);
    if(actualState->getPose() == pose && normalActionQueue->isEmpty() == FALSE){
        //        //ldbg <<"Robot is in the right position and it's doing other things."<<endl;
        constantPoseCounter++;
    } else {
        //ldbg <<"Robot is doing nothing. Setting new pose"<<endl;
        constantPoseCounter=0;
        actualState->setPose(pose);
        actualState->setTimestamp(timestamp);
        if(pastState == NULL){
            pastState = new RobotState(actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());
            pastState->setPose(pose);
            pastState->setTimestamp(timestamp);
        }
    }
}

void RobotController::checkIfStall(double comp_x, double comp_y, double comp_t)
{
    const Pose pose(comp_x, comp_y, comp_t);
    if(actualState->getPose() == pose && !actualState->isIdle()){
        stallCounter++;
        if(stallCounter > STALL_LIMIT){
            //            //ldbg <<space <<"Robot is stall." << endl;
            actualState->setStall(YES);
        }
    } else {
        stallCounter=0;
        //        //ldbg <<space <<"Robot isn't stall" << endl;
        actualState->setStall(NO);
    }
}

void RobotController::checkAround(const Data::Pose &pose)
{
    Point oldPosePoint(actualState->getPose().getX(), actualState->getPose().getY());
    Point newPosePoint(pose.getX(), pose.getY());
    // //ldbg <<"Robot Controller: Old pose is (" << oldPosePoint.x()<< ", " << oldPosePoint.y()<<" ) "<<endl;
    // //ldbg <<"Robot Controller: New pose is (" << pose.getX()<< ", " << pose.getY() <<" ) "<<endl;

    double distance = oldPosePoint.distance(newPosePoint);
    double angDist = fabs(actualState->getPose().getTheta()-pose.getTheta());

    //    //ldbg <<"Robot pose is distance from new pose of " << distance << " meters and of "<< angDist <<" radiant"<< endl;

    if(distance <= TRASL_TOL && angDist <= ANGLE_TOL && !actualState->isIdle()){
        counterAround++;
        if(counterAround > 3*STALL_LIMIT)
            //            //ldbg << "Robot is near enough."<<endl;
            isAround = true;
    } else {
        counterAround=0;
        //        //ldbg << "Robot is far away from position."<<endl;
        isAround = false;

    }
}

void RobotController::handleCameraData(const CameraData &camera)
{
    if(!streamImage)
        return;
    //Up to now, I show the image on the robot ui
    //at the end, i have to build a Wireless message,
    //specifying the robot who is sending the message
    //and the image captured.
    BuddyMessage buddyMessage(robotNameFromIndex(robotId),
                              robotNameFromIndex(BASE_STATION_ID), &camera);
    WirelessMessage wireless(&buddyMessage);
    //    //ldbg << space <<" Robot received an image!" << endl;
    emit signalWirelessMessage(wireless);
    emit cameraDataSignal(camera, actualState->getPose());
}

void RobotController::handleWaypoint(const WaypointCommand * waypoint)
{
    ldbg <<"Waypoint received at (" << waypoint->getWaypoint().getX()
        << ", " << waypoint->getWaypoint().getY() << ", "<< waypoint->getWaypoint().getTheta()<< " } "<<endl;
    ldbg <<"Actual Pose = (" << actualState->getPose().getX()<<", "
        <<actualState->getPose().getY()<<", "<<actualState->getPose().getTheta()<<")"<<endl;
    actualWaypoint = new WaypointCommand(waypoint->getWaypoint(), false, 0.0);
    stopRobot(UPDATE_STATUS);
    ldbg <<"Actual Pose = (" << actualState->getPose().getX()<<", "
        <<actualState->getPose().getY()<<", "<<actualState->getPose().getTheta()<<")"<<endl;
    emit userMoveCleanBadFrontiers();

    activateSonar = YES;
    useHybridControl = NO;
    wayPointRecv = YES;
    isWaypointNotificationNeeded = waypoint->isNotificationNeeded();
    waypointTimer = waypoint->getTimer();

    Pose waypointPose = waypoint->getWaypoint();
    double waypointAngle = computeRotationFromPoses(actualState->getPose(), waypointPose);
    ldbg <<"Angle to perform: " << fromRadiantToDegree(waypointAngle) << endl;

    //Translation
    double dx2 = pow(waypointPose.getX()- actualState->getPose().getX(), 2);
    double dy2 = pow(waypointPose.getY()- actualState->getPose().getY(), 2);
    double waypointTraslation = sqrt(dy2+dx2);
    ldbg <<"Metres to move: " << waypointTraslation << endl;
    moveRobot(fromRadiantToDegree(waypointAngle), waypointTraslation, 0.0);
}

void RobotController::insertActionToPerform(Action::ActionType type, double value)
{
    if(type == Action::Rotation){
        //ldbg << space << "Pathplanner ordered me to perform a rotation of value " << value << endl;
        moveRobot(value, 0.0, 0.0);
    } else  {
        //ldbg << space << "Pathplanner ordered me to perform a translation of value " << value << endl;
        moveRobot(0.0, value, 0.0);
    }
}

void RobotController::actionToPerform(PathPlanner::AbstractAction *action)
{
    activateSonar = YES;
    if (reactiveBehaviorStatus>DEACTIVATED)
        return;
    if(obstacleAvoidanceTimer != NULL && obstacleAvoidanceTimer->isActive()){
        obstacleAvoidanceTimer->stop();
    }
    if(randomActionTimer->isActive())
        randomActionTimer->stop();


    //ldbg << "I have received the action!"<<endl;
    if(typeid(*action) == typeid(Data::Action)){
        //ldbg << "Action of type Action"<<endl;
        Data::Action *act = (Data::Action *)action;
        insertActionToPerform(act->getType(), act->getValue());
        useHybridControl = false;
    } else if(typeid(*action) == typeid(HybridPoseAction)){
        //ldbg << "Action of type HybridPoseAction"<<endl;
        HybridPoseAction *act = (HybridPoseAction *)action;
        HybridPoseAction *toEnqueue = new HybridPoseAction(act->getValue());
        hybridActionQueue->enqueue(toEnqueue);
        useHybridControl = true;
        justStarted = true;
    }
}

//======= UTILITIES =======//

RobotState * RobotController::getActualState()
{
    return actualState;
}

void RobotController::updatePose(const Pose &pose)
{
    actualState->setPose(pose);
}

void RobotController::setStreamImageFlag(bool stream)
{
    streamMutex->lock();
    streamImage = stream;
    streamMutex->unlock();
}

void RobotController::updateSLAMPose(SLAM::TimedPose pose)
{
    //    //ldbg << "Slam pose received: " << pose << endl;
    checkIfStall(pose.getX(), pose.getY(), pose.getTheta());
    checkIfIdle(pose.getX(), pose.getY(), pose.getTheta(), pose.getTimestamp());
    checkAround(pose);
    //Emit the signal for the update of the state
    emit stateUpdate();
}

//======= ROBOT MOTION =======//
void RobotController::moveRobot(
        double angle1, double translation, double angle2)
{
    if(OPEN_LOOP == 1){
        openLoopMovement(angle1, translation, angle2);
    } else {
        //qDebug() <<"MoveRobot: OPEN_LOOP == 1"<<endl;
        odometryClosedLoop(angle1, translation, angle2);
    }
}

void RobotController::haltRobotMovement()
{
    ldbg << "haltRobotMovement: Path planner stops the robot" << endl;
    stopRobot(true);
    doMovement(0,0);
    emit restartExplorationSignal(false);
}


void RobotController::haltRobot()
{
    //    //ldbg << "haltRobotMovement: Path planner stops the robot" << endl;
    stopRobot(true);
    doMovement(0,0);
}

void RobotController::
doMovement(double leftSpeed, double rightSpeed)
{
    actualState->setLeftSpeed(leftSpeed);
    actualState->setRightSpeed(rightSpeed);
    WheelMessage command(leftSpeed, rightSpeed);
    emit launchCommand(command);
}

void RobotController::stopRobot(bool saveState)
{
    doMovement(0,0);

    if(saveState){
        saveRobotState();
    }

    while(normalActionQueue->isEmpty() == FALSE){
        delete normalActionQueue->dequeue();
    }

    while(!hybridActionQueue->isEmpty())
        delete hybridActionQueue->dequeue();

    //ldbg << "stopRobot: ActionToDo size is "<< actionsToDo->size() << endl;
    //ldbg << "stopRonot: hybridActionQueue size is "<< hybridActionQueue->size() << endl;

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

void RobotController::openLoopMovement(double angle1, double translation, double angle2)
{
    Q_UNUSED(angle1) Q_UNUSED(translation) Q_UNUSED(angle2)
            //Metodo temporaneo da togliere...
}

//======= ROBOT CONTROL =======//
void RobotController::odometryClosedLoop(double angle1, double translation, double angle2)
{
    //I build and enqueue the first action: ROTATION
    if(angle1 != 0.0){
        double radiant1 = fromDegreeToRadiants(angle1);
        Action* act1 = new Action();
        act1->setType(Action::Rotation);
        act1->setValue(radiant1);
        ldbg << "Robot Controller - odometryClosedLoop: Enqueued a rotation of " << angle1 << " radiants" << endl;
        normalActionQueue->enqueue(act1);
        isMovementStarted = TRUE;
        isMovementEnded = FALSE;
        isNewAction = TRUE;
    }
    //I build and enqueue the second action: TRANSLATION
    if(translation != 0.0){
        Action* act2 = new Action();
        act2->setType(Action::Translation);
        act2->setValue(translation);
        normalActionQueue->enqueue(act2);
        ldbg<< "Robot Controller - odometryClosedLoop: Enqueued a translation of " << translation << " meters" << endl;
        isMovementStarted = TRUE;
        isMovementEnded = FALSE;
        isNewAction = TRUE;
    }
    //I build and enqueue the third action: ROTATION
    if(angle2 != 0.0){
        double radiant3 = fromDegreeToRadiants(angle2);
        Action* act3 = new Action();
        act3->setType(Action::Rotation);
        act3->setValue(radiant3);
        normalActionQueue->enqueue(act3);
        isMovementStarted = TRUE;
        isMovementEnded = FALSE;
        isNewAction = TRUE;
        ldbg << "Robot Controller - odometryClosedLoop: Enqueued a rotation of " << angle2 << " radiants" << endl;
    }
}

bool RobotController::poseReached(const Pose &pose) const
{
    const Pose current = actualState->getPose();
    const Point toReach(pose.x(), pose.y());
    const Point robotLocation(current.x(), current.y());
    const bool directCheck = robotLocation.distance(toReach) <= INVERSE_KIN_TRASL_TOL &&
            std::abs(wrapRad(current.theta() - pose.theta())) <= INVERSE_KIN_ANG_TOL;

    if(std::isinf(previousPose.theta())) {
        /* There is no previous pose yet */
        return directCheck;
    } else {
        const LineSegment path(previousPose.x(), previousPose.y(), current.x(), current.y());
        double theta1 = current.theta(), theta2 = current.theta();
        bool angularOvershoot;
        if(theta2 < theta1) exchange(theta1, theta2);
        if(theta1 + M_PI < theta2) {
            angularOvershoot = pose.theta() >= theta2 && pose.theta() <= theta1;
        } else {
            angularOvershoot = pose.theta() >= theta1 && pose.theta() <= theta2;
        }
        return (path.intersectsCircle(toReach, INVERSE_KIN_TRASL_TOL) &&
                angularOvershoot) || directCheck;
    }
}

//======= STATES UPDATE PROCESSING =======//
void RobotController::onStateUpdated()
{
    //    qDebug() << "*******************Ricevuto segnale StateUpdate*****************************************";
    bool isIdle = actualState->isIdle();
    if(useHybridControl){

        if(!hybridActionQueue->isEmpty()){
            ldbg << "onStateUpdated: actionQueue size = " << hybridActionQueue->size() << endl;
            //peek the first action to perform
            HybridPoseAction *act = hybridActionQueue->head();
            //measure the difference with the actual pose
            Point myPose(actualState->getPose().getX(), actualState->getPose().getY());
            Point toReach(act->getValue().getX(), act->getValue().getY());

            double distance = myPose.distance(toReach);
            double angDist = wrapRad(actualState->getPose().getTheta()-(act->getValue().getTheta()));

            //ldbg << "Robot Controller - onStateUpdated:Actual State " << actualState->getPose() << "; toreach " << act->getValue() <<"; toll "<< ANGLE_TOL << endl;
            //ldbg << "Robot Controller - onStateUpdated:traslDist "<<distance<<"; angDist " << angDist << endl;
            //ldbg << "Robot Controller - onStateUpdated:actionTS " << actionStartTimestamp << "; curTS " << actualState->getTimestamp() << endl;

            bool controllerStop = almostEqual(actualState->getRightSpeed(), 0) && almostEqual(actualState->getLeftSpeed(), 0);

            if(poseReached(act->getValue()) || controllerStop) {
                //pose reached
                //ldbg << "Robot Controller - onStateUpdated:deque & delete" << endl;
                actionStartTimestamp = actualState->getTimestamp();
                act = hybridActionQueue->dequeue();
                delete act;
                saveRobotState();
                if(isAround){
                    isAround = false;
                    counterAround = 0;
                }

            }
            if(hybridActionQueue->isEmpty()){
                //ldbg << "Robot Controller - onStateUpdated: Hybrid action queue is empty!" << endl;
                actionStartTimestamp = -1;
                stopRobot(false);
            } else {
                if(actionStartTimestamp < 0)
                    actionStartTimestamp = actualState->getTimestamp();
                act = hybridActionQueue->head();
                const Data::Pose displacedPose(SLAM::Geometry::Rototranslation(actualState->getPose(), act->getValue()).vectorForm());

                pastHybridPoseToReach = displacedPose;

                double timeLeft = actionStartTimestamp + /*1.5 */ DELTA_T - actualState->getTimestamp();
                if(timeLeft < DELTA_T/5)
                    timeLeft = DELTA_T/5;

                WheelSpeeds ws = invKin->computeSpeeds(displacedPose, timeLeft);

                // //ldbg << "Robot Controller: PosetoReach = " << act->getValue() << endl;
                //  //ldbg << "Robot Controller: Speeds: left = " << ws.getLeftSpeed() <<", right = " << ws.getRightSpeed() << endl;
                if(useHybridControl == true)
                    doMovement(ws.getLeftSpeed(), ws.getRightSpeed());
                //}
            }
            previousPose = actualState->getPose();
            justStarted = NO;
        }
    } else {
        //Check if there are actions to do
        if(normalActionQueue->isEmpty() == FALSE){
            Action* todo = normalActionQueue->head();
            switch(todo->getType()){
            //==== ROTATION CASE ====//
            case Action::Rotation:
                //ldbg<<"onStateUpdated: controlRotation"<<endl;
                controlRotation(*todo);
                break;
                //==== TRANSLATION CASE ====//
            case Action::Translation:
                //ldbg<<"onStateUpdated: controlTranslationn"<<endl;
                controlTranslation(*todo);
                break;
            default:
                //I don't know how to get here, maybe an incorrect use of the Action's type field
                //For safety, I stop the robot!

                ldbg<<"Error"<<endl;
                stopRobot(true);
                break;
            }
        } else {
            //The Action Queue is empty.
            //However, it can be empty because the robot has finished a movement.
            //ldbg<<"NO Actions to do"<<endl;
            if(!isTeleoperated){
                //ldbg << "onStateUpdated:stop the robot" << endl;
                //If the robot is moving, I stop it.
                isMovementEnded = TRUE;
                isNewAction = FALSE;
                stopRobot(false);
                emit restartExplorationSignal(true);

            }
            if (wayPointRecv && reactiveBehaviorStatus == DEACTIVATED)
                notifyAfterWaypoint();
        }
        isSpeedJustChanged=FALSE;
    }
}

void RobotController::notifyAfterWaypoint()
{
    ldbg << "notifying after waypoint!" << endl;
    ldbg << wayPointRecv<<endl;
    if(wayPointRecv)
    {
        if(userEnabled)
        {
            if(waypointTimer > 0){
                QTimer::singleShot(waypointTimer*MILLIS, this, SLOT(restartExploration()));
            } else {
                restartExploration();
            }
            wayPointRecv = false;
            delete actualWaypoint;
        }
        else
        {
            //Disable sonar
            ldbg<<"Disable sonar"<<endl;
            activateSonar = false;
        }
    }


    ////ldbg << "must notify? "<<mustNotify << endl;
    if(isWaypointNotificationNeeded){
        isWaypointNotificationNeeded = false;
        stopRobot(true);
        //Create the information
        QString str(robotNameFromIndex(robotId));
        str = str.append(" has finished its movement");
        InfoMessage info(MOVEMENT_END, str);
        //Create the buddy
        BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID), &info);
        //Create the wireless
        WirelessMessage wm(&buddy);
        //Send the wireless
        emit signalWirelessMessage(wm);
        ldbg << "message waypoint movement finished" << endl;

    }
}

void RobotController::controlRotation(const Action &rotation) {
    if(isNewAction == TRUE){
        constantPoseCounter=0;
        if(rotation.getValue()>0){
            if(rotation.getValue()<= fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE) || slowMotion){
                //ldbg << "controlRotation: I rotate at slow speed clockwise!"<< endl;
                doMovement(-LOW_SPEED, LOW_SPEED);
            } else
                doMovement(-MED_SPEED, MED_SPEED);
        } else {
            if(rotation.getValue()>= -(fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE)) || slowMotion){
                //ldbg << "controlRotation: I rotate at slow speed counter-clockwise!"<< endl;
                doMovement(LOW_SPEED, -LOW_SPEED);
            }else
                doMovement(MED_SPEED, -MED_SPEED);
        }
        isSpeedChanged = FALSE;
        isNewAction = FALSE;

    } else {
        //In this case i have a rotation, but the robot is performing a rotation
        //Check if i finish the desired rotation

        ////ldbg << "Robot Controller - controlRotation: action value = "<< fromRadiantToDegree(rotation.getValue()) << endl;
        //ldbg << "Robot Controller - controlRotation: actual angle = "<< fromRadiantToDegree(actualState->getPose().getTheta())<<endl;
        double angleToReach = pastState->getPose().getTheta() + rotation.getValue();
        ////ldbg << "Robot Controller - controlRotation: angleToReachNoWrap = "<<fromRadiantToDegree(angleToReach) << endl;
        //ldbg << "Robot Controller - controlRotation: angleToReach = "<<fromRadiantToDegree(wrapRad(angleToReach)) << endl;

        angleToReach = wrapRad(angleToReach);
        double performedAngle =  actualState->getPose().getTheta() ;
        double distance = angularDistance(performedAngle, angleToReach);

        ldbg << "controlRotation: distance = "<< fromRadiantToDegree(distance)<<endl;

        if((fabs(distance) <= ANGLE_TOL && isSpeedJustChanged==FALSE)
                || constantPoseCounter >= STALL_LIMIT || countSpeedChange > STALL_LIMIT)
        {
            if(constantPoseCounter>=STALL_LIMIT || countSpeedChange > STALL_LIMIT)
            {
                ldbg<<"Robot can't move, restart!!!"<<endl;
                stopRobot(false);
                ldbg<<"Pose counter: "<<constantPoseCounter<<endl;
                ldbg<<"Speed change: "<<countSpeedChange<<endl;
                isSpeedJustChanged = TRUE;
                isNewAction = TRUE;
                countSpeedChange = 0;
                emit restartExploration();
                return;
            }

            ldbg<<"Pose counter: "<<constantPoseCounter<<endl;
            ldbg<<"Speed change: "<<countSpeedChange<<endl;
            ldbg << "controlRotation: set point reached" << endl;
            doMovement(0,0);

            delete normalActionQueue->dequeue();

            //Push the reached state into the states history
            Pose* newPose = new Pose(actualState->getPose().getX(), actualState->getPose().getY(),
                                     actualState->getPose().getTheta());
            RobotState* newState = new RobotState(*newPose, actualState->getTimestamp(),
                                                  actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());


            delete pastState;
            pastState = newState;
            setPointReachedRotation = true;

            //recall this method to check if i need to do another action
            isSpeedJustChanged = TRUE;
            isNewAction = TRUE;
            countSpeedChange = 0;
            onStateUpdated();

        } else if(fabs(distance) <= 10*ANGLE_TOL && isSpeedJustChanged==FALSE){
            //The robot is approaching the set point. So i have to decrease the speed.
            ldbg << "controlRotation: decrease the speed" << endl;
            //PRM
            double leftSpeed=actualState->getLeftSpeed();
            double rightSpeed=actualState->getRightSpeed();
            if(((fabs(leftSpeed))>MIN_SPEED)){
                leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
            }
            else{
                if(distance>0){
                    leftSpeed=-MIN_SPEED;
                }
                else{
                    leftSpeed=MIN_SPEED;
                }
            }
            if(fabs(rightSpeed)>MIN_SPEED){
                rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
            }
            else{
                if(distance>0){
                    rightSpeed=MIN_SPEED;
                }
                else{
                    rightSpeed=-MIN_SPEED;
                }
            }
            countSpeedChange++;
            doMovement(leftSpeed, rightSpeed);
        }
        //
        else if(distance < 0 && rotation.getValue() > 0 && !isSpeedJustChanged){

            //ldbg << "controlRotation: negative distance, positive rotation" << endl;
            //In this case, i've made a rotation that is higher than what I need:
            //I can send a command to change the direction, slowing down the speed
            //PRM
            if(actualState->getLeftSpeed() < 0){

                double leftSpeed=actualState->getLeftSpeed();
                double rightSpeed=actualState->getRightSpeed();
                if(((fabs(leftSpeed))>MIN_SPEED)){
                    leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
                }
                else{
                    leftSpeed=-MIN_SPEED;
                }
                if(fabs(rightSpeed)>MIN_SPEED){
                    rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
                }
                else{
                    rightSpeed=MIN_SPEED;
                }
                doMovement(-leftSpeed, -rightSpeed);
                isSpeedChanged = TRUE;
                countSpeedChange++;
                ldbg << "controlRotation: speed inverted" << endl;
            }
        } else if (distance > 0 && rotation.getValue() > 0 && !isSpeedJustChanged){
            //ldbg << "controlRotation: positive distance, positive rotation" << endl;
            //PRM
            if(actualState->getLeftSpeed() > 0){
                isSpeedChanged = true;
                countSpeedChange++;
            }
            if(distance < fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE) || slowMotion){
                doMovement(-LOW_SPEED, LOW_SPEED);
            } else {
                doMovement(-MED_SPEED, MED_SPEED);
            }
        } else if(distance > 0 && rotation.getValue() < 0 && !isSpeedJustChanged){
            //ldbg << "controlRotation: positive distance, negative rotation" << endl;
            //In this case, i've made a rotation that is higher than what I need:
            //I can send a command to change the direction, slowing down the speed
            if(actualState->getLeftSpeed() > 0){


                double leftSpeed=actualState->getLeftSpeed();
                double rightSpeed=actualState->getRightSpeed();

                if(((fabs(leftSpeed))>MIN_SPEED)){
                    leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
                }
                else{
                    leftSpeed=MIN_SPEED;
                }
                if(fabs(rightSpeed)>MIN_SPEED){
                    rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
                }
                else{
                    rightSpeed=-MIN_SPEED;
                }

                doMovement(-leftSpeed, -rightSpeed);
                //
                //} else {
                //    doMovement(actualState->getLeftSpeed()*SPEED_DECR_FACTOR, -actualState->getRightSpeed()*SPEED_DECR_FACTOR);
                //}
                ldbg << "controlRotation: speed inverted" << endl;
                isSpeedChanged = TRUE;
                countSpeedChange++;
            }
        } else if (distance < 0 && rotation.getValue() < 0 && !isSpeedJustChanged){
            ldbg << "controlRotation: negative distance, negative rotation" << endl;
            if (actualState->getLeftSpeed() < 0){
                isSpeedChanged = true;
                countSpeedChange++;
            }
            if(fabs(distance) < fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE) || slowMotion)
                doMovement(LOW_SPEED, -LOW_SPEED);
            else
                doMovement(MED_SPEED, -MED_SPEED);
        }
    }
}


void RobotController::controlTranslation(const Action &translation) {
    if(isNewAction == TRUE)
    {
        constantPoseCounter=0;
        //If it is rotating or if it is stopped
        //He must start a movement
        ldbg << "controlTranslation: Inizio traslazione. Todo vale " << translation.getValue() << endl;
        if(translation.getValue()>0)
        {
            ldbg << "controlTranslation: Devo andare avanti di: "<< translation.getValue()<<endl;
            if(translation.getValue() <= LOW_SPEED_LIMIT_TRASL){
                ldbg << "controlTranslation: I start move slowly!"<< endl;
                doMovement(MED_SPEED, MED_SPEED);
            }
            else
            {
                doMovement(HIGH_SPEED, HIGH_SPEED);
            }
        }
        else
        {
            ldbg << "controlTranslation: Devo andare indietro di: "<< translation.getValue()<<endl;
            if(translation.getValue() >= -LOW_SPEED_LIMIT_TRASL){
                ldbg << "controlTranslation: I start move slowly!"<< endl;
                doMovement(-MED_SPEED, -MED_SPEED);
            } else
                doMovement(-HIGH_SPEED, -HIGH_SPEED);
        }
        isSpeedChanged = FALSE;
        isNewAction = FALSE;

    } else {
        //It is already moving forward
        double distToCover = translation.getValue();
        double distCoveredSquare = pow(actualState->getPose().getX()-pastState->getPose().getX(), 2) +
                pow(actualState->getPose().getY()-pastState->getPose().getY(), 2);
        double distCovered = 0;
        if(distCoveredSquare > 0){
            distCovered = sqrt(distCoveredSquare);
        }
        if (distToCover>0)
            ldbg << "controlTranslation: Sto già andando avanti. Distance covered: "<< distCovered<<endl;
        else
            ldbg << "controlTranslation: Sto già andando indietro. Distance covered: "<< distCovered<<endl;

        if(((distCovered-fabs(distToCover) <= TRASL_TOL && distCovered-fabs(distToCover) >= -(TRASL_TOL)) && isSpeedJustChanged==FALSE)
                || constantPoseCounter >= 10){
            ldbg << "controlTranslation: ==> set point reached" << endl;
            //PRM

            doMovement(0.0,0.0);
            delete normalActionQueue->dequeue();

            //Push the reached state into the states history
            Pose* newPose = new Pose(actualState->getPose().getX(), actualState->getPose().getY(),
                                     actualState->getPose().getTheta());
            RobotState* newState = new RobotState(*newPose, actualState->getTimestamp(),
                                                  actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());

            delete pastState;
            pastState = newState;
            //ldbg << "controlTranslation: Spilo traslazione" << endl;
            //recall this method to check if i need to do another action
            isSpeedJustChanged = TRUE;
            isNewAction = TRUE;
            onStateUpdated();
            //PRM
        } else if ((distCovered-fabs(distToCover) <= 4*TRASL_TOL && distCovered-fabs(distToCover) >= -(4*TRASL_TOL)) && isSpeedJustChanged==FALSE) {
            //The robot is approaching the set point. So i have to decrease the speed.
            ldbg << "Approaching: distanceCovered "<<distCovered<<endl;
            ldbg << "distanceCover "<<distToCover<<endl;
            // //ldbg << "controlTranslation: I'm near to the set point. I decrease my movement speed!"<< endl;
            double leftSpeed=actualState->getLeftSpeed();
            double rightSpeed=actualState->getRightSpeed();
            //velocità troppo alta diminuisco
            if(((fabs(leftSpeed))>MIN_SPEED)){
                leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
                rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
            }
            else{
                //non siamo ancora arrivati all'obiettivo
                if((distCovered-fabs(distToCover))<0){
                    if(distToCover>0){
                        leftSpeed=MIN_SPEED;
                        rightSpeed=MIN_SPEED;
                    }
                    else{
                        leftSpeed=-MIN_SPEED;
                        rightSpeed=-MIN_SPEED;
                    }
                }
                //obiettivo superato
                else{
                    ldbg<<"Target passed"<<endl;
                    isSpeedChanged = TRUE;
                    if(distToCover>0){
                        leftSpeed=-MIN_SPEED;
                        rightSpeed=-MIN_SPEED;
                    }
                    else{
                        leftSpeed=MIN_SPEED;
                        rightSpeed=MIN_SPEED;
                    }
                }
            }

            doMovement(leftSpeed, rightSpeed);
            //
        }
        else if (fabs(distToCover)-distCovered < 0 && !isSpeedChanged){
            //The robot has covered more than the required distance.
            ldbg <<"controlTranslation: Ho superato il set point!"<<endl;
            //I go backward, reducing the speed
            //PRM
            double leftSpeed=actualState->getLeftSpeed();
            double rightSpeed=actualState->getRightSpeed();
            if(((fabs(leftSpeed))>MIN_SPEED)){
                leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
            }
            else{
                leftSpeed=MIN_SPEED;
            }
            if(fabs(rightSpeed)>=MIN_SPEED){
                rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
            }
            else{
                rightSpeed=MIN_SPEED;
            }
            doMovement(-leftSpeed, -rightSpeed);
            //
            isSpeedChanged = TRUE;
        } else if(fabs(distToCover)-distCovered > 0  && isSpeedChanged == TRUE){
            //The dual case of the above one
            ldbg <<"controlTranslation: Sono ancora troppo indietro?"<<endl;
            //PRM
            double leftSpeed=actualState->getLeftSpeed();
            double rightSpeed=actualState->getRightSpeed();
            if(abs(leftSpeed)>MIN_SPEED){
                leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
            }
            else{
                leftSpeed=-MIN_SPEED;
            }
            if(abs(rightSpeed)>MIN_SPEED){
                rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
            }
            else{
                rightSpeed=-MIN_SPEED;
            }
            doMovement(-leftSpeed, -rightSpeed);
            isSpeedChanged = FALSE;
        }
        else if(actualState->getLeftSpeed()==0){
            ldbg <<"controlTranslation: Sono fermo....reset della velocità"<<endl;
            if(translation.getValue()>0){
                if(fabs(distToCover)-distCovered > 0){
                    doMovement(HIGH_SPEED, HIGH_SPEED);
                }
                else{
                    doMovement(-HIGH_SPEED, -HIGH_SPEED);
                }
            }
            else{
                if(fabs(distToCover)-distCovered > 0){
                    doMovement(-HIGH_SPEED, -HIGH_SPEED);
                }
                else{
                    doMovement(HIGH_SPEED, HIGH_SPEED);
                }
            }
            //
        }
    }
}

void RobotController::restartExploration()
{
    //ldbg << "restartExploration: userEnabled? " << userEnabled << endl;
    if(userEnabled && reactiveBehaviorStatus == DEACTIVATED){
        ldbg <<"EMIT: restart Exploration"<<endl;
        emit restartExplorationSignal(true);
        emit restartPathPlanningSignal(true);
    }
}

void RobotController::timerPart()
{
    if (sonarObstacleTimeNumber == 0){
        //ldbg<<" First time"<<endl;
        //ldbg<< sonarObstacleTimeNumber <<endl;
        obstacleAvoidanceTimer->singleShot(60000,this,SLOT(onTimeoutObstacle()));
        //ldbg<<"Active?"<<obstacleAvoidanceTimer->isActive()<<endl;
        sonarObstacleTimeNumber++;
    }
    else
    {
        if (sonarObstacleTimeNumber>2)
        {
            //ldbg << "Too many times"<<endl;
            sonarObstacleTimeNumber = 0;
            obstacleAvoidanceTimer->stop();
            if (!hybridActionQueue->isEmpty()){
                Pose goal = hybridActionQueue->head()->getValue();
                //ldbg << "I cannot reach pose "<<goal.getX() << ","<<goal.getY()<<endl;
                normalActionQueue->clear();
                hybridActionQueue->clear();
                emit badFrontierSignal(goal);
            }
            else
            {
                //ldbg<<"ActionsToDo"<<actionsToDo->count()<<endl;
            }
        }
        else
        {
            sonarObstacleTimeNumber++;
            //ldbg<<"Sonar ha provato volte " << sonarObstacleTimeNumber<<endl;
            //ldbg<<"Il timer Ã¨ attivo? "<<obstacleAvoidanceTimer->isActive()<<endl;
        }
    }
}
void RobotController::onTimeoutObstacle()
{
    //    qDebug() << "*******************Ricevuto segnale Timeout*****************************************";
    int timer = obstacleAvoidanceTimer->interval();
    //ldbg<<"Son passati " <<timer<<endl;
    sonarObstacleTimeNumber = 0;
}

void RobotController::checkSonarStatiComplesso(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL)
{
    if (distanceLeftL < THRESHOLD)
    {
        if (distanceLeft < THRESHOLD)
        {
            if (distanceFront < THRESHOLD)
            {
                if (distanceRight < THRESHOLD)
                {
                    if (distanceRightR < THRESHOLD)
                    {
                        if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLLFRRR)
                        {
                            ////ldbg << "Caso LL_L_F_R_RR: Ostacoli ovunque"<<endl;
                            moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                            typeMovement = LLLFRRR;
                            ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLFRRR)
                        {
                            ////ldbg << "Typemovement was LLLFRRR. Now is" << typeMovement <<endl;
                            reactiveBehaviorStatus = DEACTIVATED;
                        }
                    }
                    else
                    {
                        if (reactiveBehaviorStatus == FIRSTTIME)
                        {
                            ////ldbg << "Caso LL_L_F_R: Estrema destra libera"<<endl;
                            moveRobot(0,VAI_INDIETRO,0);
                            moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                            typeMovement = LLLFR;
                            ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLFR)
                        {
                           // //ldbg << "Typemovement was LLLFR. Now is" << typeMovement <<endl;
                            reactiveBehaviorStatus = DEACTIVATED;
                        }
                    }
                }
                else
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLLF)
                    {

                       // //ldbg << "Caso LL_L_F: Destra libera"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LLLF;
                         ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLF)
                    {
                        // //ldbg << "Typemovement was LLLF. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLL)
                {

                    ////ldbg << "Caso LL_L: Sinistra occupata"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                    typeMovement = LLL;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLL)
                {
                    ////ldbg << "Typemovement was LLL. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LL)
            {

                ////ldbg << "Caso LL: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = LL;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=LL)
            {
                ////ldbg << "Typemovement was LL. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceLeft < THRESHOLD)
    {
        if (distanceFront < THRESHOLD)
        {
            if (distanceRight < THRESHOLD)
            {
                if (distanceRightR < THRESHOLD)
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LFRRR)
                    {

                       // //ldbg << "Caso L_F_R_RR: Possibile pertugio estrema sinistra"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LFRRR;
                        ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LFRRR)
                    {
                        ////ldbg << "Typemovement was LFRR. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
                else
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LFR)
                    {

                        ////ldbg << "Caso L_F_R: Estremi liberi. Vado indietro"<<endl;
                        moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                        typeMovement = LFR;
                        ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LFR)
                    {
                        ////ldbg << "Typemovement was LFR. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LF)
                {

                    ////ldbg << "Caso L_F: Destra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                    typeMovement = LF;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=LF)
                {
                    ////ldbg << "Typemovement was LF. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)
            {

               // //ldbg << "Caso L: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = L;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=L)
            {
                ////ldbg << "Typemovement was L. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }

        }
    }
    else if (distanceFront < THRESHOLD)
    {
        if (distanceRight < THRESHOLD)
        {
            if (distanceRightR < THRESHOLD)
            {
                if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=FRRR)
                {

                   // //ldbg << "Caso F_R_RR: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FRRR;
                    //ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=FRRR)
                {
                    ////ldbg << "Typemovement was FRR. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=FR)
                {

                    ////ldbg << "Caso F_R: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FR;
                   ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=FR)
                {
                    ////ldbg << "Typemovement was FR. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {

            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=F)
            {

                ////ldbg << "Caso F: Caso indecisione. Vado indietro."<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                typeMovement = F;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=F)
            {
                ////ldbg << "Typemovement was F. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceRight < THRESHOLD)
    {
        if (distanceRightR < THRESHOLD)
        {
            if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=RRR)
            {
                ////ldbg << "Caso R_RR: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = RRR;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=RRR)
            {
                //dbg << "Typemovement was RRR. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=R)
            {

                ////ldbg << "Caso R: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = R;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC  && typeMovement!=R)
            {
                ////ldbg << "Typemovement was R. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;;
            }
        }
    }
    else if (distanceRightR < THRESHOLD)
    {
        if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=RR)
        {

            ////ldbg << "Caso_RR: Sinistra libera"<<endl;
            moveRobot(0,VAI_INDIETRO,0);
            moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
            typeMovement = RR;
            ldbg<< typeMovement <<endl;
        }
        else if(reactiveBehaviorStatus == EXEC && typeMovement!=RR)
        {
            ////ldbg << "Typemovement was RR. Now is" << typeMovement <<endl;
            reactiveBehaviorStatus = DEACTIVATED;
        }
    }
    //else
        ////ldbg<<"Waiting movement typeMovement" << typeMovement << endl;
}

void RobotController::reactiveFrontCheck(const Data::SonarData &sonar)
{

    actionStartTimestamp = -1; //used by mladen to see how much time has passed to give the new control

    //ldbg << "Using front "<< sonar.getMinSonarName() << ". He say: " << sonar.getMinDistance() << endl;


    if (sonar.getMinDistance()<THRESHOLD)
    {
        ldbg<<"Front Sonar: "<<sonar.getMinSonarName()<<endl;
        ldbg<<"Obstacle found"<<endl;
        double distanceLeftL = sonar.getFront(0);
        double distanceLeft = sonar.getFront(1);
        double distanceFront = sonar.getFront(2);
        double distanceRight = sonar.getFront(3);
        double distanceRightR = sonar.getFront(4);

        ldbg << "distanceRightR vale " << distanceRightR<<endl;
        ldbg << "distanceRight vale " << distanceRight<<endl;
        ldbg << "distanceFront vale " << distanceFront<<endl;
        ldbg << "distanceLeft vale " << distanceLeft<<endl;
        ldbg << "distanceLeftL vale " << distanceLeftL<<endl;
        ldbg << "actionstToDo size "<<normalActionQueue->size()<<endl;

        time_t rawtime;
        struct tm * timeinfo;
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        ldbg<<"Current local time and date: "<< asctime(timeinfo)<<endl;


        ldbg<< "reactiveBehavior "<<reactiveBehaviorStatus<<endl;

        if ((reactiveBehaviorStatus == EXEC || reactiveBehaviorStatus == FIRSTTIME) && sonar.getMinDistance()<THRESHOLD){
            reactiveBehaviorStatus = DEACTIVATED;
        }

        if (reactiveBehaviorStatus == DEACTIVATED)
        {
            stopRobot(true);
            reactiveBehaviorStatus = FIRSTTIME;

        }
        else if (reactiveBehaviorStatus == FIRSTTIME)
        {
            reactiveBehaviorStatus = EXEC;
        }

        Pose actualPose = actualState->getPose();
        ldbg << "Actual Pose ( " << actualPose.getX() << " , " << actualPose.getY() << " , " <<fromRadiantToDegree(actualPose.getTheta())<<endl;
        ldbg<< "reactiveBehavior "<<reactiveBehaviorStatus<<endl;

        if (reactiveBehaviorStatus == FIRSTTIME)
           {

               //ldbg<<"Robot Controller: First time, actionstToDo size "<<normalActionQueue->size()<<endl;
               //ldbg<<"Typemovement = "<<typeMovement<<endl;
               lastFrontSonarData = sonar;

               if (!isKenaf)
                   slowMotion = true;
               checkSonarStatiComplesso(distanceRightR, distanceLeftL, distanceLeft, distanceFront, distanceRight);
           }
           else if (reactiveBehaviorStatus == EXEC)
           {
               //ldbg<<"Robot Controller: Execution, actionstToDo size"<<normalActionQueue->size()<<endl;
               if (!isKenaf)
                   slowMotion = true;
               checkSonarStatiComplesso(distanceRightR, distanceLeftL, distanceLeft, distanceFront, distanceRight);
           }
           useHybridControl = false;
       }
    else
    {
        //        ldbg<<"reactiveBehaviorStatus "<<reactiveBehaviorStatus<<endl;
        //        ldbg<<actionsToDo->size()<<endl;


        if (reactiveBehaviorStatus > DEACTIVATED && normalActionQueue->size()==0)
            //if (waitingSonarMessage && setPointReachedRotation && setPointReachedTranslation)
        {
            slowMotion = false;
            reactiveBehaviorStatus = DEACTIVATED;
            if(wayPointRecv)
            {
                if (wayPointCounter<10)
                {
                    ldbg<<"Ho un waypoint in memoria. Sto provando " <<wayPointCounter <<endl;
                    wayPointCounter++;
                    handleWaypoint(actualWaypoint);
                }
                else
                {
                    ldbg<<"Non posso ragggiungerlo, cazzo! "<<endl;
                    delete actualWaypoint;
                    sendSonarMessage();
                    wayPointRecv = false;
                    stopRobot(true);
                }
            }
            else
            {
                ldbg<<"oldFrontier vale ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
                if (refindPathCounter<20)
                {
                    refindPathCounter++;
                    ldbg<<"refind path"<<endl;
                    ldbg<<"actualFrontier vale ( "<<actualFrontier->x()<<" , "<<actualFrontier->y() << endl;
                    emit refindPath(actualFrontier->x(), actualFrontier->y());
                }
                else
                {
                    restartExploration();
                }

            }

            //ldbg<<"Exit: actionstToDo size"<<actionsToDo->size()<<endl;
        }
    }
}

void RobotController::reactiveBackCheck(const Data::SonarData &sonar)
{
    QString sonarName = sonar.getMinSonarName();
    if(activateSonar){
        //we are not in teleoperating

        if(sonar.getMinBackDistance()<THRESHOLD){

            //ldbg << "Using back "<< sonarName << ". He say: " << sonar.getMinBackDistance() << endl;

            ldbg<<"Back Sonar: "<<sonar.getMinSonarName()<<endl;
            ldbg<<"Obstacle found"<<endl;

            double distanceLeftL = sonar.getBack(0);
            double distanceLeft = sonar.getBack(1);
            double distanceFront = sonar.getBack(2);
            double distanceRight = sonar.getBack(3);
            double distanceRightR = sonar.getBack(4);

            ldbg << "distanceRightR vale " << distanceRightR<<endl;
            ldbg << "distanceRight vale " << distanceRight<<endl;
            ldbg << "distanceFront vale " << distanceFront<<endl;
            ldbg << "distanceLeft vale " << distanceLeft<<endl;
            ldbg << "distanceLeftL vale " << distanceLeftL<<endl;
            ldbg << "Using back "<< sonarName << ". He say: " << sonar.getMinDistance() << endl;


            if (reactiveBackBehaviorStatus == DEACTIVATED)
            {
                stopRobot(true);
                reactiveBackBehaviorStatus = FIRSTTIME;

            }
            else if (reactiveBackBehaviorStatus == FIRSTTIME)
            {
                reactiveBackBehaviorStatus = EXEC;
            }


            Pose actualPose = actualState->getPose();
            ldbg << "Actual Pose ( " << actualPose.getX() << " , " << actualPose.getY() << " , " <<fromRadiantToDegree(actualPose.getTheta())<<endl;
            ldbg<< "reactiveBehavior "<<reactiveBackBehaviorStatus<<endl;


            if (reactiveBackBehaviorStatus == FIRSTTIME)
            {
                moveRobot(0,VAI_AVANTI,0);
            }

            if (reactiveBackBehaviorStatus > DEACTIVATED && normalActionQueue->size()==0)
                //if (waitingSonarMessage && setPointReachedRotation && setPointReachedTranslation)
            {
                slowMotion = false;
                reactiveBackBehaviorStatus = DEACTIVATED;

                if(wayPointRecv)
                {
                    if (wayPointCounter<10)
                    {
                        ldbg<<"Ho un waypoint in memoria. Sto provando " <<wayPointCounter <<endl;
                        wayPointCounter++;
                    }
                    else
                    {
                        ldbg<<"Non posso ragggiungerlo, cazzo! "<<endl;
                        delete actualWaypoint;
                        sendSonarMessage();
                        wayPointRecv = false;
                        stopRobot(true);
                    }
                }
                else
                {
                    ldbg<<"oldFrontier vale ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
                    if (refindPathCounter<20)
                    {
                        refindPathCounter++;
                        ldbg<<"refind path"<<endl;
                        ldbg<<"actualFrontier vale ( "<<actualFrontier->x()<<" , "<<actualFrontier->y() << endl;
                        emit refindPath(actualFrontier->x(), actualFrontier->y());
                    }
                    else
                    {
                        stopRobot(true);
                        restartExploration();
                    }
                }
            }
        }
    }
}

void RobotController::handleSonarData(const Data::SonarData &sonar)
{
    //ldbg << "Robot Controller: LeftSpeed  = " << actualState->getLeftSpeed()<<"; " <<endl;
    //ldbg << "Robot Controller: RightSpeed = " << actualState->getRightSpeed()<<"; "<<endl;

    if (actualState->getLeftSpeed()>0 && actualState->getLeftSpeed()<0)
    {
        //ldbg << "Robot Controller: I'm rotating to right"<<endl;
        actualMovement = RIGHT;
    }
    else if (actualState->getRightSpeed() < 0 && actualState->getLeftSpeed() < 0)
    {
        //ldbg << "Robot Controller: I'm moving back"<<endl;
        actualMovement = BACK;
    }
    else if (actualState->getLeftSpeed()<0 && actualState->getLeftSpeed()>0)
    {
        //ldbg << "Robot Controller: I'm rotating to left"<<endl;
        actualMovement = LEFT;
    }
    else
    {
        //ldbg << "Robot Controller: I'm going straight" <<endl;
        actualMovement = FRONT;
    }

    //ldbg<<"Sonar position: " << sonar.getPosition() << endl;
    //ldbg<<"Sonar activated? " << activateSonar <<endl;
    //ldbg<<"Actual movement = "<<actualMovement<<endl;


    if(sonar.getPosition() == SonarData::Front && (actualMovement == FRONT || actualMovement == RIGHT || actualMovement == LEFT) && activateSonar)
    {
        //ldbg << "Robot Controller: Reactive front check"<<endl;
        reactiveFrontCheck(sonar);
    }
    if(sonar.getPosition() == SonarData::Back && actualMovement == BACK)
    {
        //ldbg << "Robot Controller: Reactive back check"<<endl;
        reactiveBackCheck(sonar);
    }

}


void RobotController::onPathNotFound(Data::Pose pose)
{
    //qDebug() << "Robot Controller: Path not found." << endl;
      //qDebug() << "Robot Controller: Actual Frontier "<< actualFrontier->getX() << " , "<<actualFrontier->getY()<<endl;
      //qDebug() << tryposeCounter<< endl;
      //qDebug() << "Robot Controller: Pose "<< pose.getX() << " , "<<pose.getY()<<endl;
    if (tryposeCounter < 3 && actualFrontier->getX() == pose.getX() && actualFrontier->getY() == pose.getY())
    {
        tryposeCounter++;
        emit userMoveCleanBadFrontiers();
    }
    else
    {
        emit badFrontierSignal(pose);
        tryposeCounter = 0;
    }
    useHybridControl = false;
    activateSonar = true;
    double angle2 = computeRotationFromPoses(actualState->getPose(), pose);
    moveRobot(fromRadiantToDegree(angle2),VAI_AVANTI,0);




}

void RobotController::goBackNoFrontiers()
{
    doMovement(-1,-1);
}

void RobotController::goStraightNoFrontiers()
{
    emit userMoveCleanBadFrontiers();
    useHybridControl = false;
    moveRobot(0,VAI_AVANTI,0);
}

void RobotController::sendSonarMessage()
{
    //Message sonar stop
    ////ldbg<<"Sto emettendo l'errorNotificationMessage::Navigation";<< endl;
    ErrorNotificationMessage error(ErrorNotificationMessage::Navigation);
    //Create the buddy
    BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID),
                       BuddyMessage::ErrorNotification ,&error);
    //Create the wireless
    WirelessMessage wm(&buddy);
    //Send the wireless
    emit signalWirelessMessage(wm);
    //    //ldbg << "Sonar message emitted" << endl;
}

void RobotController::recomputePath()
{
    emit refindPath(goalToRecompute->getValue().x(), goalToRecompute->getValue().y());
}

void RobotController::obstacleAvoidanceTimerExpired()
{
    //    //ldbg << "obstacleAvoidanceTimer expired, the obstacle is still there, rebuilding path" << endl;
    if(!hybridActionQueue->isEmpty()){
        HybridPoseAction *goal = hybridActionQueue->last();
        emit refindPath(goal->getValue().x(), goal->getValue().y());
        randomActionTimer->singleShot(RH_RANDOM_ACTION_TIME*MILLIS, this, SLOT(performRandomAction()));
        //add obstacle to the map of the SLAM
        //   //ldbg << "Obstacle created: " << obstacle << endl;
    }
}

void RobotController::performRandomAction()
{
    double minXPose = actualState->getPose().x()-RH_HALF_SQUARE_SIDE;
    double maxXPose = actualState->getPose().x()+RH_HALF_SQUARE_SIDE;
    double minYPose = actualState->getPose().y()-RH_HALF_SQUARE_SIDE;
    double maxYPose = actualState->getPose().y()+RH_HALF_SQUARE_SIDE;

    Point myPoint(actualState->getPose().x(), actualState->getPose().y());
    //    //ldbg << "performRandomAction: Rectangle - bottomLeft: "<<minXPose<<", "<<minYPose<<"; topRight: "<<maxXPose<<", "<<maxYPose<<";"<<endl;
    int end = 0;
    bool isReachable = false;
    double randomX = Shared::Random::uniform(minXPose, maxXPose);
    double randomY = Shared::Random::uniform(minYPose, maxYPose);

    while(!isReachable){
        if(end >= 100)
            break;
        //        //ldbg << "performRandomAction: sampled point: "<<randomX<<", "<< randomY<<";"<<endl;

        //Ask to SLAM if it reachable
        if (isKenaf)
            isReachable = slam->getMap(true).isReachable(myPoint, Point(randomX, randomY), KENAF_RADIUS);
        else
            isReachable = slam->getMap(true).isReachable(myPoint, Point(randomX, randomY), P3AT_RADIUS);
        randomX = Shared::Random::uniform(minXPose, maxXPose);
        randomY = Shared::Random::uniform(minYPose, maxYPose);
        end++;
    }
    if(isReachable){
        //        //ldbg << "performRandomAction: winning point: "<<randomX<<", "<< randomY<<";"<<endl;
        double desiredTheta = computeRotationFromPoses(actualState->getPose(),
                                                       Pose(randomX, randomY, 0));
        Pose p(randomX, randomY, actualState->getPose().getTheta()+desiredTheta);
        WaypointCommand cmd(p,false, 0);
        wayPointCounter = 0;
        handleWaypoint(&cmd);
        end++;
    }
}

void RobotController::setSlamModule(SLAM::SLAMModule *slam)
{
    this->slam = slam;
}

void RobotController::setStatus(bool status)
{
    this->userEnabled = status;
    if(status){
        ldbg << "setStatus: enabling things..." << endl;
        emit restartExplorationSignal(true);
        emit restartPathPlanningSignal(true);
    } else {
        ldbg << "setStatus: disabling things..." << endl;
        emit restartExplorationSignal(false);
        emit restartPathPlanningSignal(false);
        haltRobotMovement();
    }
}

Data::Pose RobotController::forwardKinematics(const Data::Pose &from, double vr, double vl)
{
    const double dt = DELTA_T, L = WHEEL_BASE;
    double x, y, theta;
    if(almostEqual(vl, vr)) {
        x = from.x() - vr * DELTA_T * sin(from.theta());
        y = from.y() + vr * DELTA_T * cos(from.theta());
        theta = from.theta();
    } else {
        const double R = .5 * L * (vr + vl) / (vr - vl);
        const double w = (vr - vl) / L;
        x = from.x() + R * std::cos(from.theta() + w * dt) - R * std::cos(from.theta());
        y = from.y() + R * std::sin(from.theta() + w * dt) - R * std::sin(from.theta());
        theta = wrapRad(from.theta() + w * dt);
    }
    return Data::Pose(x, y, theta);
}

void RobotController::saveActualFrontier(const Pose frontier){
    //ldbg <<"Sto salvando frontier e vale ("<<frontier.getX() << " , "<<frontier.getY()<<endl;

    if (frontier.getX() != oldFrontier->getX() && frontier.getY() != oldFrontier->getY())
    {
        delete actualFrontier;
        actualFrontier = new Pose(frontier.getX(),frontier.getY(), frontier.getTheta());
        delete oldFrontier;
        oldFrontier = new Pose(actualFrontier->getX(),actualFrontier->getY(), actualFrontier->getTheta());
        //ldbg<<"oldFrontier vale ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
        refindPathCounter = 0;
    }
}

void RobotController::waypointForKenaf(double x, double y)
{
    WaypointCommand wpCmd(Pose(x, y, 0.0), false, 0.0);
    wayPointCounter = 0;
    handleWaypoint(&wpCmd);
}
