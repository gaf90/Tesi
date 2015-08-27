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
    newAction(FALSE),
    wayPointCounter(0),
    isSpeedChanged(FALSE),
    isJustChanged(FALSE),
    normalActionQueue(new QQueue<Action *>()),
    hybridActionQueue(new QQueue<HybridPoseAction *>()),
    actionStartTimestamp(-1),
    pastState(NULL),
    actualState(new RobotState(0.0, 0.0, MAX_BATTERY)),
    robotId(id),
    lastBatterySent(MAX_BATTERY),
    streamImage(true),
    streamMutex(new QMutex()),
    isNotificationNeeded(false),
    haveReceivedWaypoint(false),
    waitTime(0),
    explorationModuleState(true),        //PRM
    reactiveFrontBehaviorStatus(DEACTIVATED),
    reactiveBackBehaviorStatus(DEACTIVATED),
    constantPoseCounter(0),
    stallCounter(0),
    sonarObstacleTimeNumber(0),
    controlRobotType(NORMAL),
    previousPose(INFINITY, INFINITY, INFINITY),
    countSpeedChange(0),
    obstacleAvoidanceTimer(new QTimer()),
    randomActionTimer(new QTimer()),
    teleOperationTimer(new QTimer()),
    sonarStatus(OFF),
    teleoperationStatus(OFF),
    userEnabled(true),      //PRM
    goalToRecompute(NULL)

{

    //ldbg << "RobotController: Start module"<<endl;

    inverseKinematicmModule = new InverseKinematicCF(WHEEL_BASE,aisKenaf);

    if (aisKenaf)
        robotType = KENAF;
    else
        robotType = P3AT;

    //Make connections
    connect(this, SIGNAL(sigRobotStateUpdated()), this, SLOT(onStateUpdated()), Qt::QueuedConnection);
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
    delete inverseKinematicmModule;
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

    if (explorationModuleState && userEnabled)
    {
        sonarStatus = ON;
        teleoperationStatus = OFF;

        randomActionTimer->stop();
        obstacleAvoidanceTimer->stop();
        onRestartExploration();
    }
}

//======= SENSORS HANDLING =======//


void RobotController::onSensorData(const Message &data)
{
    if(typeid(data) == typeid(const CameraData &))
    {

        const CameraData &camera = (const CameraData &)data;
        handleCameraData(camera);
    }
    else if(typeid(data) == typeid(const StateData &))
    {
        const StateData &state = (const StateData &)data;
        handleStateData(state);
    }
    else if(typeid(data) == typeid(const SonarData &))
    {
        const SonarData &sonar = (const SonarData &)data;
        if (OBSTACLE_EMPIRIC)
            handleEmpiricSonarData(sonar);
        else if (OBSTACLE_DYNAMIC)
            handleDynamicWindowSonarData(sonar);
        else if (OBSTACLE_NEURAL)
            handleNeuralSonarData(sonar);
    }
    else if(typeid(data) == typeid(const WirelessMessage &))
    {
        handleWirelessData(data);
    }
}

void RobotController::handleCameraData(const CameraData &camera)
{
    if(!streamImage)
        return;

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
    actualState->setBattery(state.getBattery());
    if(state.getBattery() == lastBatterySent - BATTERY_ONE_PERCENT)
    {
        lastBatterySent = state.getBattery();
        InfoMessage infoMessage(BATTERY_STATUS, QString::number(lastBatterySent));
        BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID), &infoMessage );
        WirelessMessage wirelessMessage(&buddy);

        //Send battery message to Wireless Driver
        emit sigWirelessMessageRCM(wirelessMessage);

    }
}

void RobotController::handleEmpiricSonarData(const Data::SonarData &sonar)
{
    ////ldbg << "Robot Controller: LeftSpeed  = " << actualState->getLeftSpeed()<<"; ";
    ////ldbg << " RightSpeed = " << actualState->getRightSpeed()<<"; ";

    actualMovement = (movementStateEnum)getActualMovement(actualState->getLeftSpeed(),actualState->getRightSpeed());

    ldbg<<"Robot Controller: Actual movement = "<< actualMovement<<endl;

    if(sonar.getPosition() == SonarData::Front && !(actualMovement == BACK))
    {
        //ldbg << "Robot Controller - empiric: Check front sonar."<<endl;
        handleFrontSonarData(sonar);
    }
    else if(sonar.getPosition() == SonarData::Back && actualMovement == BACK)
    {
        //ldbg << "Robot Controller - empiric: Check back sonar."<<endl;
        handleBackSonarData(sonar);
    }

}

QVector<QPair<double, double> > RobotController::getLocalReachableSearchSpace(QVector<QPair<QPair<double,double>,int> > localMap)
{
    QVector<QPair<double,double> > searchSpace (SEARCH_SPACE_GRANULARITY);
    for (int i=0; i<SEARCH_SPACE_GRANULARITY; i++)
    {
        QPair<double,double> nearestPoint = localMap[i].first;
        Pose *nearestPose = new Pose(nearestPoint.first,nearestPoint.second, i);
        WheelSpeeds ws = inverseKinematicmModule->computeSpeeds(*nearestPose,0.5);
        searchSpace[i].first = ws.getLeftSpeed();
        searchSpace[i].second = ws.getRightSpeed();
    }

    return searchSpace;
}

QVector<QPair<double,double> > RobotController::calculateSearchSpace(const Data::SonarData &sonar)
{
    //Get local set with nearest pose reachable for each index (angle).
    QVector<QPair<QPair<double,double>,int> > localMap = getLocalMap(actualState->getPose());
    QVector<QPair<double,double> > searchSpace = getLocalReachableSearchSpace(localMap);
    return searchSpace;

}

int RobotController::calculateBestVelocity(QVector<QPair< double,double> > searchSpace)
{
    double distance = 0;
    double targetHeading = 0;
    double clearance = 0;
    int bestValue = 0;
    double bestCost = 0;

    QVector<QPair<QPair<double,double>,int> > localMap = getLocalMap(actualState->getPose());

    for(int counter = 0; counter < searchSpace.size()-1;counter++)
    {
        int dx = actualFrontier->getX()-localMap[counter].first.first;
        int dy = actualFrontier->getY()-localMap[counter].first.second;
        distance = sqrt(dx^2 -dy^2);
        targetHeading = abs(actualFrontier->getTheta() - counter);
        clearance = searchSpace[counter].first;
        double cost = distance + clearance - targetHeading;
        if (cost>=bestCost)
        {
            bestCost = cost;
            bestValue = counter;
        }
    }



    return bestValue;
}

QVector<QPair<QPair<double,double>,int> > RobotController::getLocalMap(const Pose actualPose)
{
    QVector<QPair<QPair<double,double>,int> > localMap (SEARCH_SPACE_GRANULARITY);
    double actualX = actualPose.getX();
    double actualY = actualPose.getY();
    double radius = RADIUS_LOCAL;

    //Search for all the direction (angle) the nearest pose that is reachable by the robot
    for (double angle=0; angle<SEARCH_SPACE_GRANULARITY; angle++)
    {
        //Started from pose I go far until I find a pose unreachable
        for (double step=0; step<1; step+=0.1)
        {
            double deltaX = actualX + step*radius*cos(angle);
            double deltaY = actualY + step*radius*sin(angle);
            Point* deltaPosePoint = new Point(deltaX, deltaY);
            Point* actualPosePoint = new Point(actualX, actualY);
            bool isReachable = slam->getMap().isReachable(*actualPosePoint, *deltaPosePoint, P3AT_RADIUS);
            if (isReachable)
            {
                localMap[angle].second = radius/step;
                localMap[angle].first.first = deltaX;
                localMap[angle].first.second = deltaY;
            }
        }
    }
    return localMap;
}

void RobotController::handleDynamicWindowSonarData(const Data::SonarData &sonar)
{

    double leftSpeed = actualState->getLeftSpeed();
    double rightSpeed = actualState->getRightSpeed();
    Pose actualPose = actualState->getPose();
    //ldbg << "Robot Controller - dynamic: LeftSpeed  = " << leftSpeed <<"; " <<endl;
    //ldbg << "Robot Controller - dynamic: RightSpeed = " << rightSpeed<<"; "<<endl;
    //ldbg << "Robot Controller - dynamic: I'm in (" << actualPose.getX() << ", " <<
    //actualPose.getY() << ")" << endl;

    actionStartTimestamp = -1;

    actualMovement = (movementStateEnum)getActualMovement(actualState->getLeftSpeed(),actualState->getRightSpeed());

    Pose predictedPose = forwardKinematics(actualPose,leftSpeed, rightSpeed);

    //ldbg << "Robot Controller - dynamic: predicted pose is (" << predictedPose.getX() << ", " <<
    // predictedPose.getY() << ")" << endl;

    Point* predictedPosePoint = new Point(predictedPose.getX(), predictedPose.getY());
    Point* actualPosePoint = new Point(actualPose.getX(),actualPose.getY());

    bool isReachable = slam->getMap().isReachable(*actualPosePoint,*predictedPosePoint,P3AT_RADIUS);
    if (!isReachable)
    {
        lastFrontSonarData = sonar;

        //ldbg << "Robot Controller - dynamic: Start DWA."<<endl;

        stopRobot(true);

        QVector<QPair< double,double> > searchSpace = calculateSearchSpace(sonar);

        int bestValue = calculateBestVelocity(searchSpace);
        if (bestValue == -1)
        {
            //ldbg << "RobotController - dynamic: DWA failed." <<endl;
            moveRobot(0,-VAI_AVANTI,0);
        }
        else
        {
            Pose bestPose = forwardKinematics(actualPose,searchSpace[bestValue].first,searchSpace[bestValue].second);
            double trasl = bestPose.getDistance(actualPose);
            //ldbg << "RobotController - dynamic: I must traslate of " << trasl << endl;
            double rot = computeRotationFromPoses(actualPose,bestPose);
            //ldbg << "RobotController - dynamic: I must rotate of " << rot << endl;
            moveRobot(rot, trasl,0);
        }
        controlRobotType = NORMAL;

    }
    else
    {
        //ldbg<<"Robot controller - dynamic: No obstacle. ok! Normal action queue size is " << normalActionQueue->size() <<
        //endl;

        if (normalActionQueue->size()==0)
        {
            if(haveReceivedWaypoint)
                tryReachWaypoint();
            else
                tryRefindPathFrontier();
        }
    }


}

void RobotController::handleNeuralSonarData(const Data::SonarData &sonar)
{

}

void RobotController::handleFrontSonarData(const Data::SonarData &sonar)
{
    actionStartTimestamp = -1; //used by mladen to see how much time has passed to give the new control
    ////ldbg << "Robot Controller: Using front "<< sonar.getMinSonarName() << ". He say: " << sonar.getMinDistance() << endl;

    if (sonar.getMinDistance()<THRESHOLD)
        handleFrontObstacle(sonar);
    else
    {
        if (normalActionQueue->isEmpty() && reactiveFrontBehaviorStatus > DEACTIVATED)
        {
            reactiveFrontBehaviorStatus = DEACTIVATED;
            ldbg <<"Robot controller: Nothing to do. Restart exploration!" << endl;
            onRestartExploration();
        }
    }
}

int RobotController::getActualMovement(double leftSpeed, double rightSpeed)
{
    if (normalActionQueue->isEmpty())
    {
        if (leftSpeed >0 && rightSpeed<0)
            return RIGHT;
        if (leftSpeed<0 && rightSpeed>0)
            return LEFT;
        if (leftSpeed<0 && rightSpeed<0)
            return BACK;
        else
            return FRONT;
    }
    else
    {
        Action *actualAction = normalActionQueue->head();
        int type = actualAction->getType();
        double value = actualAction->getValue();

        ldbg << "Robot Controller: Next action to do is" << type << "with value " << value<<endl;

        if (type == Action::Rotation && value>0)
        {
            ldbg << "Robot Controller: I'm rotating to right!"<<endl;
            return RIGHT;
        }
        else if (type == Action::Rotation && value<0)
        {
            ldbg << "Robot Controller: I'm rotating to left!"<<endl;
            return LEFT;
        }
        else if (type == Action::Translation && value<0)
        {
            ldbg << "Robot Controller: I'm moving backward!"<<endl;
            return BACK;
        }
        else
        {
            ldbg << "Robot Controller: I'm going straight" <<endl;
            return FRONT;
        }
    }
}


void RobotController::handleFrontObstacle(const Data::SonarData &sonar)
{
    double distanceLeftL = sonar.getFront(0);
    double distanceLeft = sonar.getFront(1);
    double distanceFront = sonar.getFront(2);
    double distanceRight = sonar.getFront(3);
    double distanceRightR = sonar.getFront(4);

    if (reactiveFrontBehaviorStatus == DEACTIVATED)
    {
        stopRobot(true);
        //ldbg << "RobotController - Empiric FSM: DEACTIVATED to FIRSTTIME."<<endl;
        reactiveFrontBehaviorStatus = FIRSTTIME;
    }

    if (reactiveFrontBehaviorStatus > DEACTIVATED)
    {
        lastFrontSonarData = sonar;

        if (robotType == P3AT)
            slowMotion = true;

        obstacleAvoidanceEmpiricHandler(distanceRightR, distanceLeftL, distanceLeft, distanceFront, distanceRight);
    }

    controlRobotType = NORMAL;
}

void RobotController::tryReachWaypoint()
{
    if (wayPointCounter<3)
    {
        //ldbg<<"Robot Controller: I have a waypoint. I'm trying " <<wayPointCounter <<endl;
        wayPointCounter++;
        handleWaypoint(actualWaypoint);
    }
    else
    {
        //ldbg<<"Robot Controller: I can't reach the waypoint."<<endl;
        delete actualWaypoint;
        sendSonarMessage();
        haveReceivedWaypoint = false;
        stopRobot(true);
    }
}

void RobotController::tryRefindPathFrontier()
{
    //ldbg<<"Robot Controller: OldFrontier is ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
    if (refindPathCounter<3 ||!(actualState->getPose().getX()==actualFrontier->y() &&actualState->getPose().getY()==actualFrontier->x()))
    {
        refindPathCounter++;
        //ldbg<<"Robot Controller: Tempt number "<< refindPathCounter <<". Refind path to ("<<actualFrontier->x()<<" , "<<actualFrontier->y() << endl;
        emit sigRestartExplorationRCM(actualFrontier->x(), actualFrontier->y());
    }
    else
    {
        //ldbg<<"Robot Controller: restart exploration."<<endl;
        onRestartExploration();
    }
}

void RobotController::obstacleAvoidanceEmpiricHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL)
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
                        if (reactiveFrontBehaviorStatus == FIRSTTIME)
                        {
                            reactiveFrontBehaviorStatus = EXEC;
                            ldbg <<"Caso LL_L_F_R_RR: Ostacoli ovunque"<<endl;
                            moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                            typeMovement = LLLFRRR;
                            //ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LLLFRRR)
                        {
                            //ldbg << "Typemovement was LLLFRRR. Now is" << typeMovement <<endl;
                            reactiveFrontBehaviorStatus = DEACTIVATED;
                        }
                    }
                    else
                    {
                        if (reactiveFrontBehaviorStatus == FIRSTTIME)
                        {
                            reactiveFrontBehaviorStatus = EXEC;
                            ldbg <<"Caso LL_L_F_R: Estrema destra libera"<<endl;
                            moveRobot(0,VAI_INDIETRO,0);
                            moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                            typeMovement = LLLFR;
                            //ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LLLFR)
                        {
                            //ldbg << "Typemovement was LLLFR. Now is" << typeMovement <<endl;
                            reactiveFrontBehaviorStatus = DEACTIVATED;
                        }
                    }
                }
                else
                {
                    if (reactiveFrontBehaviorStatus == FIRSTTIME)
                    {
                        reactiveFrontBehaviorStatus = EXEC;
                        ldbg <<"Caso LL_L_F: Destra libera"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LLLF;
                        //ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LLLF)
                    {
                        //ldbg << "Typemovement was LLLF. Now is" << typeMovement <<endl;
                        reactiveFrontBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveFrontBehaviorStatus == FIRSTTIME)
                {
                    reactiveFrontBehaviorStatus = EXEC;
                    ldbg <<"Caso LL_L: Sinistra occupata"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                    typeMovement = LLL;
                    //ldbg<< typeMovement <<endl;
                }
                else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LLL)
                {
                    //ldbg << "Typemovement was LLL. Now is" << typeMovement <<endl;
                    reactiveFrontBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveFrontBehaviorStatus == FIRSTTIME)
            {
                reactiveFrontBehaviorStatus = EXEC;
                ldbg <<"Caso LL: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = LL;
                //ldbg<< typeMovement <<endl;
            }
            else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LL)
            {
                //ldbg << "Typemovement was LL. Now is" << typeMovement <<endl;
                reactiveFrontBehaviorStatus = DEACTIVATED;
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
                    if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=LFRRR)
                    {
                        reactiveFrontBehaviorStatus = EXEC;
                        ldbg <<"Caso L_F_R_RR: Possibile pertugio estrema sinistra"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LFRRR;
                        //ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LFRRR)
                    {
                        //ldbg << "Typemovement was LFRR. Now is" << typeMovement <<endl;
                        reactiveFrontBehaviorStatus = DEACTIVATED;
                    }
                }
                else
                {
                    if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=LFR)
                    {
                        reactiveFrontBehaviorStatus = EXEC;
                        ldbg <<"Caso L_F_R: Estremi liberi. Vado indietro"<<endl;
                        moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                        typeMovement = LFR;
                        //ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LFR)
                    {
                        //ldbg << "Typemovement was LFR. Now is" << typeMovement <<endl;
                        reactiveFrontBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=LF)
                {
                    reactiveFrontBehaviorStatus = EXEC;
                    ldbg <<"Caso L_F: Destra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                    typeMovement = LF;
                    //ldbg<< typeMovement <<endl;
                }
                else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=LF)
                {
                    //ldbg << "Typemovement was LF. Now is" << typeMovement <<endl;
                    reactiveFrontBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveFrontBehaviorStatus == FIRSTTIME)
            {
                reactiveFrontBehaviorStatus = EXEC;
                ldbg <<"Caso L: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = L;
                //ldbg<< typeMovement <<endl;
            }
            else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=L)
            {
                //ldbg << "Typemovement was L. Now is" << typeMovement <<endl;
                reactiveFrontBehaviorStatus = DEACTIVATED;
            }

        }
    }
    else if (distanceFront < THRESHOLD)
    {
        if (distanceRight < THRESHOLD)
        {
            if (distanceRightR < THRESHOLD)
            {
                if (reactiveFrontBehaviorStatus == FIRSTTIME)//&& typeMovement!=FRRR)
                {
                    reactiveFrontBehaviorStatus = EXEC;
                    ldbg <<"Caso F_R_RR: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FRRR;
                    //ldbg<< typeMovement <<endl;
                }
                else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=FRRR)
                {
                    //ldbg << "Typemovement was FRR. Now is" << typeMovement <<endl;
                    reactiveFrontBehaviorStatus = DEACTIVATED;
                }
            }
            else
            {
                if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=FR)
                {
                    reactiveFrontBehaviorStatus = EXEC;
                    ldbg <<"Caso F_R: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FR;
                    //ldbg<< typeMovement <<endl;
                }
                else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=FR)
                {
                    //ldbg << "Typemovement was FR. Now is" << typeMovement <<endl;
                    reactiveFrontBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {

            if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=F)
            {
                reactiveFrontBehaviorStatus = EXEC;
                ldbg <<"Caso F: Caso indecisione. Vado indietro."<<endl;
                moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MIN);
                typeMovement = F;
                //ldbg<< typeMovement <<endl;
            }
            else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=F)
            {
                //ldbg << "Typemovement was F. Now is" << typeMovement <<endl;
                reactiveFrontBehaviorStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceRight < THRESHOLD)
    {
        if (distanceRightR < THRESHOLD)
        {
            if (reactiveFrontBehaviorStatus == FIRSTTIME)//&& typeMovement!=RRR)
            {
                reactiveFrontBehaviorStatus = EXEC;
                ldbg <<"Caso R_RR: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = RRR;
                //ldbg<< typeMovement <<endl;
            }
            else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=RRR)
            {
                //ldbg << "Typemovement was RRR. Now is" << typeMovement <<endl;
                reactiveFrontBehaviorStatus = DEACTIVATED;
            }
        }
        else
        {
            if (reactiveFrontBehaviorStatus == FIRSTTIME)// && typeMovement!=R)
            {
                reactiveFrontBehaviorStatus = EXEC;
                ldbg <<"Caso R: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = R;
                //ldbg<< typeMovement <<endl;
            }
            else if(reactiveFrontBehaviorStatus == EXEC  && typeMovement!=R)
            {
                //ldbg << "Typemovement was R. Now is" << typeMovement <<endl;
                reactiveFrontBehaviorStatus = DEACTIVATED;;
            }
        }
    }
    else if (distanceRightR < THRESHOLD)
    {
        if (reactiveFrontBehaviorStatus == FIRSTTIME)//&& typeMovement!=RR)
        {
            reactiveFrontBehaviorStatus = EXEC;
            ldbg <<"Caso_RR: Sinistra libera"<<endl;
            moveRobot(0,VAI_INDIETRO,0);
            moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
            typeMovement = RR;
            //ldbg<< typeMovement <<endl;
        }
        else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=RR)
        {
            //ldbg << "Typemovement was RR. Now is" << typeMovement <<endl;
            reactiveFrontBehaviorStatus = DEACTIVATED;
        }
    }
    else
        ldbg<<"Waiting movement typeMovement" << typeMovement << endl;
}

void RobotController::handleBackObstacle(const Data::SonarData &sonar)
{

    double distanceLeftL = sonar.getBack(0);
    double distanceLeft = sonar.getBack(1);
    double distanceFront = sonar.getBack(2);
    double distanceRight = sonar.getBack(3);
    double distanceRightR = sonar.getBack(4);

    //ldbg << "Robot Controller: BRR vale " << distanceRightR<<endl;
    //ldbg << "Robot Controller: BR vale " << distanceRight<<endl;
    //ldbg << "Robot Controller: B " << distanceFront<<endl;
    //ldbg << "Robot Controller: BL " << distanceLeft<<endl;
    //ldbg << "Robot Controller: BLL " << distanceLeftL<<endl;


    if (reactiveBackBehaviorStatus == DEACTIVATED)
    {
        stopRobot(true);
        //ldbg << "RobotController: DEACTIVATED to FIRSTTIME."<<endl;
        reactiveBackBehaviorStatus = FIRSTTIME;
    }

    Pose actualPose = actualState->getPose();
    //ldbg << "Robot Controller: Actual Pose ( " << actualPose.getX() << " , " << actualPose.getY() << " , " <<fromRadiantToDegree(actualPose.getTheta())<<endl;
    //ldbg<< "Robot Controller: reactiveBehavior"<<reactiveBackBehaviorStatus<<endl;


    if (reactiveBackBehaviorStatus == FIRSTTIME)
    {
        reactiveBackBehaviorStatus = EXEC;
        moveRobot(0,VAI_AVANTI,0);
    }
}

void RobotController::handleBackSonarData(const Data::SonarData &sonar)
{
    QString sonarName = sonar.getMinSonarName();
    //ldbg << "Robot Controller: Using back "<< sonarName << ". He say: " << sonar.getMinBackDistance() << endl;
    if(sonar.getMinBackDistance()<THRESHOLD)
        handleBackObstacle(sonar);
    else if (reactiveBackBehaviorStatus  == DEACTIVATED && normalActionQueue->size()==0)
    {
        onRestartExploration();
    }
}

void RobotController::handleWheelMotionMessage(const BuddyMessage *buddy)
{
    {
        const WheelMessage *wheelMessage = buddy->getWheelMessage();
        stopRobot(true);

        sonarStatus = OFF;
        teleoperationStatus = ON;
        if(almostEqual(wheelMessage->getLeftWheelSpeed(),0,0.01) && almostEqual(wheelMessage->getRightWheelSpeed(),0,0.01)){
            if(teleOperationTimer->isActive()){
                teleOperationTimer->stop();
            }
            //ldbg << "handleWirelessData. restart timers"<<endl;
            crono.start();
            teleOperationTimer->start(TELEOPERATION_TIMEOUT_MSEC);
        }
        emit sigChangeStatetExplorationRCM(false);
        emit sigChangeStatePathPlanningRCM(false);
        emit sigCleanBadFrontierRCM();

        doMovement(wheelMessage->getLeftWheelSpeed(), wheelMessage->getRightWheelSpeed());
    }
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

void RobotController::checkIfIdle(double comp_x, double comp_y, double comp_t, double timestamp)
{
    const Pose pose(comp_x, comp_y, comp_t);
    if(actualState->getPose() == pose && !normalActionQueue->isEmpty()){
        ////ldbg <<"RobotController: Robot is in the right position and it's doing other things."<<endl;
        constantPoseCounter++;
    } else {
        ////ldbg <<"Robot is doing nothing. Setting new pose"<<endl;
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
            ////ldbg <<space <<"Robot is stall." << endl;
            actualState->setStall(true);
        }
    } else {
        stallCounter=0;
        ////ldbg <<space <<"Robot isn't stall" << endl;
        actualState->setStall(false);
    }
}

void RobotController::checkAround(const Data::Pose &pose)
{
    Point oldPosePoint(actualState->getPose().getX(), actualState->getPose().getY());
    Point newPosePoint(pose.getX(), pose.getY());

    ////ldbg <<"Robot Controller: Old pose is (" << oldPosePoint.x()<< ", " << oldPosePoint.y()<<" ) "<<endl;
    ////ldbg << "Robot Controller: New pose is (" << pose.getX()<< ", " << pose.getY() <<" ) "<<endl;

    double distance = oldPosePoint.distance(newPosePoint);
    double angDist = fabs(actualState->getPose().getTheta()-pose.getTheta());

    ////ldbg <<"Robot Controller: Robot pose is distance from new pose of " << distance << " meters and of "<< angDist <<" radiant"<< endl;

    if(distance <= TRASL_TOL && angDist <= ANGLE_TOL && !actualState->isIdle()){
        counterAround++;
        if(counterAround > 3*STALL_LIMIT)
        {
            ////ldbg << "Robot Controller: Robot is near enough."<<endl;
            robotAround = NEAR_TO_POSE;
        }
    } else {
        counterAround = 0;
        ////ldbg << "Robot Controller: Robot is far away from position."<<endl;
        robotAround = FAR_TO_POSE;

    }
}

void RobotController::handleWaypoint(const WaypointCommand * waypoint)
{
    //ldbg <<"Robot Controller: Waypoint received at (" << waypoint->getWaypoint().getX()
    //<< ", " << waypoint->getWaypoint().getY() << ", "<< waypoint->getWaypoint().getTheta()<< " } "<<endl;
    //ldbg <<"Robot Controller: Actual Pose = (" << actualState->getPose().getX()<<", "
    //<<actualState->getPose().getY()<<", "<<actualState->getPose().getTheta()<<")"<<endl;

    actualWaypoint = new WaypointCommand(waypoint->getWaypoint(), false, 0.0);

    stopRobot(true);

    //ldbg <<"Robot Controller: Actual Pose = (" << actualState->getPose().getX()<<", "
    //<<actualState->getPose().getY()<<", "<<actualState->getPose().getTheta()<<")"<<endl;

    emit sigCleanBadFrontierRCM();

    sonarStatus = ON;
    controlRobotType = NORMAL;
    haveReceivedWaypoint = true;
    isNotificationNeeded = waypoint->isNotificationNeeded();
    waitTime = waypoint->getTimer();
    Pose pointToReach = waypoint->getWaypoint();
    double angle2 = computeRotationFromPoses(actualState->getPose(), pointToReach);
    //ldbg <<"Robot Controller: Angle to perform: " << fromRadiantToDegree(angle2) << endl;

    //Translation
    double dx2 = pow(pointToReach.getX()- actualState->getPose().getX(), 2);
    double dy2 = pow(pointToReach.getY()- actualState->getPose().getY(), 2);
    double trasl = sqrt(dy2+dx2);
    //ldbg <<"Robot Controller: Metres to move: " << trasl << endl;
    moveRobot(fromRadiantToDegree(angle2), trasl, 0.0);
}


void RobotController::insertActionToPerform(Action::ActionType type, double value)
{
    if(type == Action::Rotation){
        //ldbg << "RobotController: Pathplanner ordered me to perform a rotation of value " << value << endl;
        moveRobot(value, 0.0, 0.0);
    } else  {
        //ldbg << "RobotController: Pathplanner ordered me to perform a translation of value " << value << endl;
        moveRobot(0.0, value, 0.0);
    }
}

void RobotController::onPerformActionRCM(PathPlanner::AbstractAction *action)
{
    sonarStatus = ON;
    if (reactiveFrontBehaviorStatus == EXEC && normalActionQueue->size()!=0)
    {
        ldbg << "Robot Controller: Ignore perform action" << endl;
        return;
    }

    if(obstacleAvoidanceTimer != NULL && obstacleAvoidanceTimer->isActive()){
        obstacleAvoidanceTimer->stop();
    }

    if(randomActionTimer->isActive())
        randomActionTimer->stop();


    //ldbg << "Robot Controller: I have received the action!"<<endl;
    if(typeid(*action) == typeid(Data::Action)){
        //ldbg << "Robot Controller: Action of type Action"<<endl;
        Data::Action *act = (Data::Action *)action;
        insertActionToPerform(act->getType(), act->getValue());
        controlRobotType = NORMAL;
    } else if(typeid(*action) == typeid(HybridPoseAction)){
        //ldbg << "Robot Controller: Action of type HybridPoseAction"<<endl;
        HybridPoseAction *act = (HybridPoseAction *)action;
        HybridPoseAction *toEnqueue = new HybridPoseAction(act->getValue());
        hybridActionQueue->enqueue(toEnqueue);
        controlRobotType = HYBRID;
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

void RobotController::onNewRobotPose(SLAM::TimedPose pose)
{
    //ldbg << "Robot Controller: Slam pose received: " << pose << endl;

    checkIfStall(pose.getX(), pose.getY(), pose.getTheta());
    checkIfIdle(pose.getX(), pose.getY(), pose.getTheta(), pose.getTimestamp());
    checkAround(pose);
    //Emit the signal for the update of the state
    emit sigRobotStateUpdated();
}

//======= ROBOT MOTION =======//
void RobotController::moveRobot(double angle1, double translation, double angle2)
{
    odometryClosedLoop(angle1, translation, angle2);
}

void RobotController::onStopRobotForPlanning()
{
    //ldbg << "Robot Controller: Path planner stops the robot. Restart Exploration" << endl;
    stopRobot(true);
    emit sigChangeStatetExplorationRCM(false);
}


void RobotController::onRobotNotReachable()
{
    //ldbg << "Robot Controller:  Path planner stops the robot. No restart exploration." << endl;
    stopRobot(true);
}

void RobotController::doMovement(double leftSpeed, double rightSpeed)
{
    actualState->setLeftSpeed(leftSpeed);
    actualState->setRightSpeed(rightSpeed);
    WheelMessage command(leftSpeed, rightSpeed);
    emit sigDriverMessageRobot(command);
}

void RobotController::stopRobot(bool saveState)
{
    //ldbg << "RobotController: stop robot."<<endl;

    doMovement(0,0);
    typeMovement = S;

    if(saveState){
        saveRobotState();
    }

    while(normalActionQueue->isEmpty() == FALSE){
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


//======= ROBOT CONTROL =======//
void RobotController::odometryClosedLoop(double angle1, double translation, double angle2)
{
    //I build and enqueue the first action: ROTATION
    if(angle1 != 0.0){
        double radiant1 = fromDegreeToRadiants(angle1);
        Action* act1 = new Action();
        act1->setType(Action::Rotation);
        act1->setValue(radiant1);
        ldbg << "Robot Controller - odometryClosedLoop: Enqueued a rotation of " << fromRadiantToDegree(radiant1) << " grades" << endl;
        normalActionQueue->enqueue(act1);
        newAction = TRUE;
    }
    //I build and enqueue the second action: TRANSLATION
    if(translation != 0.0){
        Action* act2 = new Action();
        act2->setType(Action::Translation);
        act2->setValue(translation);
        normalActionQueue->enqueue(act2);
        ldbg<< "Robot Controller - odometryClosedLoop: Enqueued a translation of " << translation << " meters" << endl;
        newAction = TRUE;
    }
    //I build and enqueue the third action: ROTATION
    if(angle2 != 0.0){
        double radiant3 = fromDegreeToRadiants(angle2);
        Action* act3 = new Action();
        act3->setType(Action::Rotation);
        act3->setValue(radiant3);
        normalActionQueue->enqueue(act3);
        newAction = TRUE;
        ldbg << "Robot Controller - odometryClosedLoop: Enqueued a rotation of " << fromRadiantToDegree(angle2) << " grades" << endl;
    }
}

bool RobotController::poseReached(const Pose &pose) const
{
    const Pose current = actualState->getPose();
    const Point toReach(pose.x(), pose.y());
    const Point robotLocation(current.x(), current.y());
    const bool directCheck = robotLocation.distance(toReach) <= INVERSE_KIN_TRASL_TOL &&
            std::abs(wrapRad(current.theta() - pose.theta())) <= INVERSE_KIN_ANG_TOL;

    if(std::isinf(previousPose.theta()))
    {
        /* There is no previous pose yet */
        return directCheck;
    }
    else
    {
        const LineSegment path(previousPose.x(), previousPose.y(), current.x(), current.y());
        double theta1 = current.theta(), theta2 = current.theta();
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
        return (path.intersectsCircle(toReach, INVERSE_KIN_TRASL_TOL) &&
                angularOvershoot) || directCheck;
    }
}

//======= STATES UPDATE PROCESSING =======//

void RobotController::onStateUpdated()
{
    //ldbg << "Robot Controller - onStateUpdate: start";

    bool isIdle = actualState->isIdle();

    if(controlRobotType == HYBRID && !hybridActionQueue->isEmpty())
        onStateUpdatedHybrid();
    else if (controlRobotType == NORMAL && !normalActionQueue->isEmpty())
        onStateUpdatedNormal(isIdle);
    else
    {
        if(isIdle && teleoperationStatus == OFF)
        {
            newAction = FALSE;
            stopRobot(false);
            emit sigChangeStatetExplorationRCM(true);
        }
        if (haveReceivedWaypoint && reactiveFrontBehaviorStatus == DEACTIVATED)
            notifyAfterWaypoint();
    }
    isJustChanged = FALSE;
}

void RobotController::onStateUpdatedHybrid()
{

    //ldbg << "Robot Controller - onStateUpdated: actionQueue size = " << hybridActionQueue->size() << endl;

    //peek the first action to perform
    HybridPoseAction *act = hybridActionQueue->head();

    bool robotStopped = almostEqual(actualState->getRightSpeed(), 0) && almostEqual(actualState->getLeftSpeed(), 0);

    if(poseReached(act->getValue()) || robotStopped)
    {
        //pose reached
        //ldbg << "Robot Controller - onStateUpdated: deque & delete" << endl;
        actionStartTimestamp = actualState->getTimestamp();
        act = hybridActionQueue->dequeue();
        delete act;
        saveRobotState();
        if(robotAround == NEAR_TO_POSE)
        {
            robotAround = FAR_TO_POSE;
            counterAround = 0;
        }

    }
    if(hybridActionQueue->isEmpty())
    {
        //ldbg << "Robot Controller - onStateUpdated: The queue is empty!" << endl;
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

        WheelSpeeds ws = inverseKinematicmModule->computeSpeeds(displacedPose, timeLeft);

        //ldbg << "Robot Controller: action value = " << act->getValue() << endl;
        //ldbg << "Robot Controller: Speeds: left = " << ws.getLeftSpeed() <<", right = " << ws.getRightSpeed() << endl;
        if(controlRobotType == HYBRID)
            doMovement(ws.getLeftSpeed(), ws.getRightSpeed());
    }

    previousPose = actualState->getPose();
}

void RobotController::onStateUpdatedNormal(bool isIdle)
{
    Action* todo = normalActionQueue->head();
    switch(todo->getType())
    {
    //==== ROTATION CASE ====//
    case Action::Rotation:
        //ldbg<<"Robot Controller - onStateUpdated: controlRotation"<<endl;
        controlRotation(*todo);
        break;
        //==== TRANSLATION CASE ====//
    case Action::Translation:
        //ldbg<<"Robot Controller - onStateUpdated: controlTranslation"<<endl;
        controlTranslation(*todo);
        break;
    default:
        //I don't know how to get here, maybe an incorrect use of the Action's type field
        //For safety, I stop the robot!

        //ldbg<<"Robot Controller - Error"<<endl;
        stopRobot(true);
        break;
    }
}


void RobotController::notifyAfterWaypoint()
{
    //ldbg << "Robot Controller - notifying after waypoint!" << endl;
    //ldbg << haveReceivedWaypoint<<endl;
    if(haveReceivedWaypoint)
    {
        if(userEnabled)
        {
            if(waitTime > 0){
                QTimer::singleShot(waitTime*MILLIS, this, SLOT(onRestartExploration()));
            } else {
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

void RobotController::controlRotationNewAction(const Action &rotation)
{
    constantPoseCounter = 0;

    if(rotation.getValue()> 0)
    {
        ldbg << "Robot Controller - ControlRotation: I rotate clockwise!"<< endl;
        if(rotation.getValue()<= fromDegreeToRadiants(SPEED_LIMIT_ANGLE) || slowMotion)
            doMovement(-LOW_SPEED, LOW_SPEED);
        else
            doMovement(-MED_SPEED, MED_SPEED);
    }
    else
    {
        ldbg << "Robot Controller - ControlRotation: I rotate counter-clockwise!"<< endl;
        if(rotation.getValue()>= -(fromDegreeToRadiants(SPEED_LIMIT_ANGLE)) || slowMotion)
            doMovement(LOW_SPEED, -LOW_SPEED);
        else
            doMovement(MED_SPEED, -MED_SPEED);
    }
    isSpeedChanged = FALSE;
    newAction = FALSE;
}

void RobotController::controlRotationRobotStall()
{
    stopRobot(false);
    //ldbg<<"Robot Controller - Pose counter: "<<constantPoseCounter<<endl;
    //ldbg<<"Robot Controller - Speed change: "<<countSpeedChange<<endl;
    isJustChanged = TRUE;
    newAction = TRUE;
    countSpeedChange = 0;
    onRestartExploration();
}

void RobotController::controlRotationSetPointReached()
{
    //ldbg << "Robot Controller - Control Rotation: set point reached!" << endl;

    doMovement(0,0);
    typeMovement = S;
    delete normalActionQueue->dequeue();

    //Push the reached state into the states history
    Pose* newPose = new Pose(actualState->getPose().getX(), actualState->getPose().getY(),
                             actualState->getPose().getTheta());
    RobotState* newState = new RobotState(*newPose, actualState->getTimestamp(),
                                          actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());


    delete pastState;
    pastState = newState;

    isJustChanged = TRUE;
    newAction = TRUE;
    countSpeedChange = 0;
    onStateUpdated();
}

void RobotController::controlRotationNegPos()
{
    if(actualState->getLeftSpeed() < 0)
    {
        double leftSpeed = actualState->getLeftSpeed();
        double rightSpeed = actualState->getRightSpeed();

        if(((fabs(leftSpeed))>MED_SPEED))
            leftSpeed = leftSpeed*SPEED_DECR_FACTOR;
        else
            leftSpeed = -MED_SPEED;
        if(fabs(rightSpeed)>MED_SPEED)
            rightSpeed = rightSpeed*SPEED_DECR_FACTOR;
        else
            rightSpeed = MED_SPEED;

        doMovement(-leftSpeed, -rightSpeed);
        ldbg << "Robot Controller - controlRotation: speed inverted (counter-clockwise)" << endl;
        isSpeedChanged = TRUE;
        countSpeedChange++;
    }
}

void RobotController::controlRotationPosPos(double distance)
{
    //PRM
    if(actualState->getLeftSpeed() > 0)
    {
        isSpeedChanged = true;
        countSpeedChange++;
    }

    if(distance < fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE) || slowMotion){
        doMovement(-LOW_SPEED, LOW_SPEED);
    } else {
        doMovement(-MED_SPEED, MED_SPEED);
    }
}

void RobotController::controlRotationPosNeg()
{
    //In this case, i've made a rotation that is higher than what I need:
    //I can send a command to change the direction, slowing down the speed
    if(actualState->getLeftSpeed() > 0)
    {

        double leftSpeed=actualState->getLeftSpeed();
        double rightSpeed=actualState->getRightSpeed();

        if(((fabs(leftSpeed))>MED_SPEED))
            leftSpeed=leftSpeed*SPEED_DECR_FACTOR;
        else
            leftSpeed=MED_SPEED;
        if(fabs(rightSpeed)>MED_SPEED)
            rightSpeed=rightSpeed*SPEED_DECR_FACTOR;
        else
            rightSpeed=-MED_SPEED;

        doMovement(-leftSpeed, -rightSpeed);
        ldbg << "Robot Controller - controlRotation: speed inverted (clockwise)" << endl;
        isSpeedChanged = TRUE;
        countSpeedChange++;
    }
}

void RobotController::controlRotationNegNeg(double distance)
{
    if (actualState->getLeftSpeed() < 0)
    {
        isSpeedChanged = true;
        countSpeedChange++;
    }
    if(fabs(distance) < fromDegreeToRadiants(LOW_SPEED_LIMIT_ANGLE) || slowMotion)
        doMovement(LOW_SPEED, -LOW_SPEED);
    else
        doMovement(MED_SPEED, -MED_SPEED);
}

void RobotController::controlRotation(const Action &rotation) {
    if(newAction)
        controlRotationNewAction(rotation);
    else
    {
        //In this case i have a rotation, but the robot is performing a rotation
        //Check if i finish the desired rotation
        ldbg << "Robot Controller - ControlRotation: Action value = " << fromRadiantToDegree(rotation.getValue()) << endl;
        ldbg << "Robot Controller - controlRotation: Actual angle = " << fromRadiantToDegree(actualState->getPose().getTheta())<<endl;

        double angleToReach = pastState->getPose().getTheta() + rotation.getValue();

        ldbg << "Robot Controller - controlRotation: Real angle to reach = "<< fromRadiantToDegree(angleToReach) << endl;

        angleToReach = wrapRad(angleToReach);

        double performedAngle =  actualState->getPose().getTheta();
        double distance = angularDistance(performedAngle, angleToReach);

        ldbg << "Robot Controller - controlRotation: Angular distance = "<< distance <<endl;

        if((fabs(distance) <= ANGLE_TOL && !isJustChanged) || constantPoseCounter >= STALL_LIMIT || countSpeedChange > STALL_LIMIT)
        {
            if(constantPoseCounter>=STALL_LIMIT || countSpeedChange > STALL_LIMIT)
            {
                ldbg<<"Robot Controller - Robot can't move, restart!!!"<<endl;
                controlRotationRobotStall();
                return;
            }
            //ldbg<<"Robot Controller - Pose counter: "<<constantPoseCounter<<endl;
            controlRotationSetPointReached();

        }
        else if(distance < 0 && rotation.getValue() > 0 && !isJustChanged)
            controlRotationNegPos();
        else if (distance > 0 && rotation.getValue() > 0 && !isJustChanged)
            controlRotationPosPos(distance);
        else if(distance > 0 && rotation.getValue() < 0 && !isJustChanged)
            controlRotationPosNeg();
        else if (distance < 0 && rotation.getValue() < 0 && !isJustChanged)
            controlRotationNegNeg(distance);
    }
}

void RobotController::controlTraslationNewAction(const Action &traslation)
{
    constantPoseCounter = 0;

    //ldbg << "Robot Controller - controlTranslation: Inizio traslazione. Todo vale " << todo.getValue() << endl;
    if(traslation.getValue()>0)
    {
        //ldbg << "Robot Controller - controlTranslation: Devo andare avanti di: "<< todo.getValue()<<endl;
        if(traslation.getValue() <= LOW_SPEED_LIMIT_TRASL)
            doMovement(MED_SPEED, MED_SPEED);
        else
            doMovement(HIGH_SPEED, HIGH_SPEED);
    }
    else
    {
        //ldbg << "Robot Controller - controlTranslation: Devo andare indietro di: "<< todo.getValue()<<endl;
        if(traslation.getValue() >= -LOW_SPEED_LIMIT_TRASL)
            doMovement(-MED_SPEED, -MED_SPEED);
        else
            doMovement(-HIGH_SPEED, -HIGH_SPEED);
    }
    isSpeedChanged = FALSE;
    newAction = FALSE;
}

void RobotController::controlTraslationSetPointReached()
{

    doMovement(0.0,0.0);
    typeMovement = S;

    delete normalActionQueue->dequeue();

    //Push the reached state into the states history
    Pose* newPose = new Pose(actualState->getPose().getX(), actualState->getPose().getY(),
                             actualState->getPose().getTheta());
    RobotState* newState = new RobotState(*newPose, actualState->getTimestamp(),
                                          actualState->getRightSpeed(), actualState->getLeftSpeed(), actualState->getBattery());

    delete pastState;
    pastState = newState;

    isJustChanged = TRUE;
    newAction = TRUE;
    onStateUpdated();
}

void RobotController::controlTraslationTooFar()
{
    //I go backward, reducing the speed
    //PRM
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

    doMovement(-leftSpeed, -rightSpeed);
    isSpeedChanged = TRUE;
}

void RobotController::controlTraslationTooBack()
{
    //PRM
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

    doMovement(leftSpeed, rightSpeed);
    isSpeedChanged = FALSE;
}

void RobotController::controlTranslation(const Action &traslation)
{
    if(newAction)
        controlTraslationNewAction(traslation);
    else
    {
        //It is already moving forward
        double distToCover = traslation.getValue();
        double distCoveredSquare = pow(actualState->getPose().getX()- pastState->getPose().getX(), 2) +
                pow(actualState->getPose().getY()-pastState->getPose().getY(), 2);
        double distCovered = 0;
        if(distToCover > 0)
            distCovered = sqrt(distCoveredSquare);
        else
            distCovered = -sqrt(distCoveredSquare);

        ldbg<<"RobotController - controlTraslation: I covered " << distCovered <<". I must cover : " << distToCover - distCovered << endl;

        if(((distToCover - distCovered <= TRASL_TOL && distToCover - distCovered >= -TRASL_TOL) && isJustChanged==FALSE) || constantPoseCounter >= 10)
        {
            ldbg<<"RobotController - controlTraslation: Set point reached!"<<endl;
            controlTraslationSetPointReached();
        }
        else if (distToCover - distCovered < -TRASL_TOL && !isSpeedChanged)
        {
            ldbg <<"RobotController - controlTranslation: I'm going too far' !"<<endl;
            controlTraslationTooFar();
        }
        else if(distToCover - distCovered > TRASL_TOL  && !isSpeedChanged)
        {
            ldbg <<"RobotController - controlTranslation: continue movement."<<endl;
            controlTraslationTooBack();
        }
    }
}

void RobotController::onRestartExploration()
{
    //ldbg << "Robot Controller - onRestartExploration: userEnabled? " << userEnabled << endl;
    if(userEnabled)
    {
        ldbg <<"Robot Controller: restart Exploration"<<endl;
        emit sigChangeStatetExplorationRCM(true);
        emit sigChangeStatePathPlanningRCM(true);
    }
}

void RobotController::timerPart()
{
    if (sonarObstacleTimeNumber == 0){
        //ldbg<<" Robot Controller - First time"<<endl;
        //ldbg<< sonarObstacleTimeNumber <<endl;
        obstacleAvoidanceTimer->singleShot(5000,this,SLOT(onTimeoutObstacle()));
        //ldbg<<"Robot Controller: obstacle Aovidance Timer. Active?"<<obstacleAvoidanceTimer->isActive()<<endl;
        sonarObstacleTimeNumber++;
    }
    else
    {
        if (sonarObstacleTimeNumber>2)
        {
            //ldbg << "Robot Controller - Too many times"<<endl;
            sonarObstacleTimeNumber = 0;
            obstacleAvoidanceTimer->stop();
            if (!hybridActionQueue->isEmpty()){
                Pose goal = hybridActionQueue->head()->getValue();
                //ldbg << "Robot Controller - I cannot reach pose "<<goal.getX() << ","<<goal.getY()<<endl;
                normalActionQueue->clear();
                hybridActionQueue->clear();
                emit sigHandleBadFrontierRCM(goal);
            }
            else
            {
                //ldbg<<"ActionsToDo"<<normalActionQueue->count()<<endl;
            }
        }
        else
        {
            sonarObstacleTimeNumber++;
            //ldbg<<"Robot Controller - Sonar ha provato volte " << sonarObstacleTimeNumber<<endl;
            //ldbg<<"Robot Controller - Il timer ? attivo? "<<obstacleAvoidanceTimer->isActive()<<endl;
        }
    }
}
void RobotController::onTimeoutObstacle()
{
    qDebug() << "*******************Ricevuto segnale Timeout*****************************************";
    int timer = obstacleAvoidanceTimer->interval();
    //ldbg<<"Robot Controller - Son passati " <<timer<<endl;
    sonarObstacleTimeNumber = 0;
}

void RobotController::onHandleBadFrontierRCM(Data::Pose pose)
{
    qDebug() << "Robot Controller - On path" << endl;
    qDebug() << "Robot Controller - Actual Frontier "<< actualFrontier->getX() << " , "<<actualFrontier->getY()<<endl;
    qDebug() << tryposeCounter<< endl;
    qDebug() << "Robot Controller - Pose "<< pose.getX() << " , "<<pose.getY()<<endl;
    if (tryposeCounter < 3 && actualFrontier->getX() == pose.getX() && actualFrontier->getY() == pose.getY())
    {
        tryposeCounter++;
        emit sigCleanBadFrontierRCM();
    }
    else
    {
        emit sigHandleBadFrontierRCM(pose);
        tryposeCounter = 0;
    }
    controlRobotType = NORMAL;
    sonarStatus = ON;
    double angle2 = computeRotationFromPoses(actualState->getPose(), pose);
    moveRobot(fromRadiantToDegree(angle2),1,0);




}


void RobotController::onNoFrontierAvailableRCM()
{
    emit sigCleanBadFrontierRCM();
    controlRobotType = NORMAL;;
    moveRobot(0,VAI_AVANTI,0);
}

void RobotController::sendSonarMessage()
{
    //Message sonar stop
    //ldbg<<"Robot Controller - Sto emettendo l'errorNotificationMessage::Navigation"<< endl;
    ErrorNotificationMessage error(ErrorNotificationMessage::Navigation);
    //Create the buddy
    BuddyMessage buddy(robotNameFromIndex(robotId), robotNameFromIndex(BASE_STATION_ID),
                       BuddyMessage::ErrorNotification ,&error);
    //Create the wireless
    WirelessMessage wm(&buddy);
    //Send the wireless
    emit sigWirelessMessageRCM(wm);
    //ldbg << "Robot Controller - Sonar message emitted" << endl;
}

void RobotController::recomputePath()
{
    emit sigRestartExplorationRCM(goalToRecompute->getValue().x(), goalToRecompute->getValue().y());
}

void RobotController::onObstacleAvoidanceTimerExpired()
{
    //ldbg << "Robot Controller - obstacleAvoidanceTimer expired, the obstacle is still there, rebuilding path" << endl;
    if(!hybridActionQueue->isEmpty()){
        HybridPoseAction *goal = hybridActionQueue->last();
        emit sigRestartExplorationRCM(goal->getValue().x(), goal->getValue().y());
        randomActionTimer->singleShot(RH_RANDOM_ACTION_TIME*MILLIS, this, SLOT(onPerformRandomAction()));
        SLAM::Geometry::LineSegment obstacle = Rototranslation(actualState->getPose()) * lastFrontSonarData.getSensedObstacle();
        //add obstacle to the map of the SLAM
        //ldbg << "Obstacle created: " << obstacle << endl;
    }
}

void RobotController::onPerformRandomAction()
{
    double minXPose = actualState->getPose().x()-RH_HALF_SQUARE_SIDE;
    double maxXPose = actualState->getPose().x()+RH_HALF_SQUARE_SIDE;
    double minYPose = actualState->getPose().y()-RH_HALF_SQUARE_SIDE;
    double maxYPose = actualState->getPose().y()+RH_HALF_SQUARE_SIDE;

    Point myPoint(actualState->getPose().x(), actualState->getPose().y());
    //    ////ldbg << "onPerformRandomAction: Rectangle - bottomLeft: "<<minXPose<<", "<<minYPose<<"; topRight: "<<maxXPose<<", "<<maxYPose<<";"<<endl;
    int end = 0;
    bool isReachable = false;
    double randomX = Shared::Random::uniform(minXPose, maxXPose);
    double randomY = Shared::Random::uniform(minYPose, maxYPose);

    while(!isReachable){
        if(end >= 100)
            break;
        //        ////ldbg << "onPerformRandomAction: sampled point: "<<randomX<<", "<< randomY<<";"<<endl;

        //Ask to SLAM if it reachable
        if (robotType == KENAF)
            isReachable = slam->getMap(true).isReachable(myPoint, Point(randomX, randomY), KENAF_RADIUS);
        else
            isReachable = slam->getMap(true).isReachable(myPoint, Point(randomX, randomY), P3AT_RADIUS);
        randomX = Shared::Random::uniform(minXPose, maxXPose);
        randomY = Shared::Random::uniform(minYPose, maxYPose);
        end++;
    }
    if(isReachable){
        //        ////ldbg << "onPerformRandomAction: winning point: "<<randomX<<", "<< randomY<<";"<<endl;
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
        //ldbg << "Robot Controller - setStatus: enabling things..." << endl;
        emit sigChangeStatetExplorationRCM(true);
        emit sigChangeStatePathPlanningRCM(true);
    } else {
        //ldbg << "Robot Controller - setStatus: disabling things..." << endl;
        emit sigChangeStatetExplorationRCM(false);
        emit sigChangeStatePathPlanningRCM(false);
        onStopRobotForPlanning();
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

void RobotController::onFrontierToReachRCM(const Pose frontier)
{
    //ldbg <<"Robot Controller - Sto salvando frontier e vale ("<<frontier.getX() << " , "<<frontier.getY()<<endl;

    if (frontier.getX() != oldFrontier->getX() && frontier.getY() != oldFrontier->getY())
    {
        delete actualFrontier;
        actualFrontier = new Pose(frontier.getX(),frontier.getY(), frontier.getTheta());
        delete oldFrontier;
        oldFrontier = new Pose(actualFrontier->getX(),actualFrontier->getY(), actualFrontier->getTheta());
        //ldbg<<"Robot Controller - oldFrontier vale ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
        refindPathCounter = 0;
    }
}

void RobotController::onPointToReachRCM(double x, double y)
{
    WaypointCommand wpCmd(Pose(x, y, 0.0), false, 0.0);
    wayPointCounter = 0;
    handleWaypoint(&wpCmd);
}
