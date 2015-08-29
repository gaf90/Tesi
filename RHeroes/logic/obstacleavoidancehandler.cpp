#include "obstacleavoidancehandler.h"
#include "wheelspeeds.h"
#include <data/robotstate.h>
#include <slam/geometry/point.h>
#include <slam/slammodule.h>
#include <shared/constants.h>
#include <shared/utilities.h>


using namespace Data;
using namespace SLAM::Geometry;



ObstacleAvoidance::ObstacleAvoidance(InverseKinematic *inverseKinematic, QObject *parent) : QObject(parent)
{

    reactiveFrontBehaviorStatus = DEACTIVATED;
    reactiveBackBehaviorStatus = DEACTIVATED;
    this->inverseKinematicModule = inverseKinematic;

}

void ObstacleAvoidance::setSlamModule(SLAM::SLAMModule *slam)
{
    this->slam = slam;
}

void ObstacleAvoidance::setMovementType(int type)
{
    this->typeMovement = (typeMovementEnum)type;
}

void ObstacleAvoidance::handleObstacle(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    if (OBSTACLE_EMPIRIC)
        handleEmpiricSonarData(sonar,actualState,actualAction,actualFrontier);
    else if (OBSTACLE_DYNAMIC)
        handleDynamicWindowSonarData(sonar,actualState,actualAction,actualFrontier);
    else if (OBSTACLE_NEURAL)
        handleNeuralSonarData(sonar,actualState,actualAction,actualFrontier);
}

bool ObstacleAvoidance::isReachablePose(Pose predictedPose, Pose actualPose)
{
    Point* predictedPosePoint = new Point(predictedPose.getX(), predictedPose.getY());
    Point* actualPosePoint = new Point(actualPose.getX(),actualPose.getY());

    bool isReachable = slam->getMap(true).isReachable(*actualPosePoint,*predictedPosePoint,P3AT_RADIUS);

    return isReachable;
}

void ObstacleAvoidance::handleDynamicWindowSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier)
{

    this->actualAction = actualAction;
    this->actualFrontier = actualFrontier;
    this->actualState = actualState;

    double leftSpeed = actualState->getLeftSpeed();
    double rightSpeed = actualState->getRightSpeed();

    if (leftSpeed != 0 && rightSpeed != 0)
    {
        emit sigChangeActionStartTimestamp(-1);

        Pose actualPose = actualState->getPose();
        actualMovement = (movementStateEnum)getActualMovement(leftSpeed,rightSpeed);


        Pose predictedPose = forwardKinematics(actualPose,leftSpeed, rightSpeed,DYNAMIC_DELTA_T);

        ldbg <<"Obstacle Avoidance - dynamic: predicted pose is (" << predictedPose.getX() << ", " <<predictedPose.getY() << ")" << endl;

        if (!isReachablePose(predictedPose, actualPose) ||
                (sonar.getPosition() == SonarData::Front && actualMovement == FRONT && sonar.getMinDistance()<THRESHOLD))
        {
            emit sigUpdateSonarData(sonar);
            emit sigStopRobot(true);

            ldbg <<"Obstacle Avoidance - dynamic: Start DWA. Actual Pose is ("<<
                   actualPose.getX() << ", " << actualPose.getY() <<", " << actualPose.getTheta() <<")"<<endl;

            QVector<QPair< double,double> > searchSpace = calculateSearchSpace(sonar);

            int bestValue = calculateBestVelocity(searchSpace);
            if (bestValue == -1)
            {
                ldbg <<"Obstacle Avoidance - dynamic: DWA failed." <<endl;
                emit sigMoveRobot(0,-VAI_AVANTI,0);
            }
            else
            {
                Pose bestPose = forwardKinematics(actualPose,searchSpace[bestValue].first,searchSpace[bestValue].second,DYNAMIC_DELTA_T);
                ldbg <<"Obstacle Avoidance - dynamic: Best pose is (" << bestPose.getX() << ", " << bestPose.getY() << endl;
                double trasl = bestPose.getDistance(actualPose);
                ldbg <<"Obstacle Avoidance - dynamic: I must traslate of " << trasl << endl;
                double rot = computeRotationFromPoses(actualPose,bestPose);
                ldbg <<"Obstacle Avoidance - dynamic: I must rotate of " << rot << endl;
                emit sigMoveRobot(rot, trasl,0);
            }
            emit sigChangeRobotControlType(NORMAL);

        }
        else
        {
            //ldbg<<"Robot controller - dynamic: No obstacle. ok!"<<endl;
        }
    }
}

QVector<QPair<double,double> > ObstacleAvoidance::calculateSearchSpace(const Data::SonarData &sonar)
{

    Pose actualPose = actualState->getPose();
    QVector<LocalMapEl> localMap = getLocalMap(actualPose);
    QVector<QPair<double,double> > searchSpace = getLocalReachableSearchSpace(localMap);
    return searchSpace;

}

QVector<ObstacleAvoidance::LocalMapEl> ObstacleAvoidance::getLocalMap(const Pose actualPose)
{
    QVector<LocalMapEl> localMap (SEARCH_SPACE_GRANULARITY);
    double actualX = actualPose.getX();
    double actualY = actualPose.getY();
    double radius = RADIUS_LOCAL;
    double i = 0;
    bool foundObstacle = false;

    //Search for all the direction (angle) the nearest pose that is reachable by the robot
    for (double angle=0; angle<360; angle+=360/SEARCH_SPACE_GRANULARITY)
    {
        //Initial values.
        localMap[i].angle = angle;
        localMap[i].distance = 0;
        localMap[i].x = actualX;
        localMap[i].y = actualY;

        //Started from pose I go far until I find a pose unreachable
        for (double step=0; step<1 && !foundObstacle; step+=0.1)
        {
            //ldbg << "Angle = "<<angle << " Cos = " << cos(angle) << " Sen = " << sin(angle)
            //    << "Actual X = " << actualX << "Actual Y = " << actualY;

            double deltaX = actualX + step*radius*cos(angle);
            double deltaY = actualY + step*radius*sin(angle);

            Pose deltaPose(deltaX,deltaY,angle);

            if (isReachablePose(actualPose,deltaPose))
            {
                localMap[i].distance = step*radius;
                localMap[i].x = deltaX;
                localMap[i].y = deltaY;
            }
            else
            {
                foundObstacle = true;
            }
        }
        foundObstacle = false;
        i++;
    }

//    //Print local map
//    for (double i=0; i<SEARCH_SPACE_GRANULARITY; i++)
//        ldbg << "Obstacle Avoidance - dynamic: Angle = "<< localMap[i].angle << ", X = " << localMap[i].x
//             << " , Y = " << localMap[i].y << ", Distance = " << localMap[i].distance << endl;


    return localMap;
}

QVector<QPair<double, double> > ObstacleAvoidance::getLocalReachableSearchSpace(QVector<LocalMapEl> localMap)
{

    QVector<QPair<double,double> > searchSpace (SEARCH_SPACE_GRANULARITY);
    for (int i=0; i<SEARCH_SPACE_GRANULARITY; i++)
    {
        if (localMap[i].distance == 0)
        {
            searchSpace[i].first = 0;
            searchSpace[i].second = 0;
        }
        else
        {
            double dX = (localMap[i].x - actualState->getPose().getX())*cos(actualState->getPose().getTheta());
            double curvature = 2*dX/localMap[i].distance;

            double leftSpeed = (MED_SPEED/RH_RADIUS)*(1 - (WHEEL_BASE/2)*curvature);
            double rightSpeed = (MED_SPEED/RH_RADIUS)*(1 + (WHEEL_BASE/2)*curvature);

            //ldbg << "Obstacle Avoidance - dynamic: Pose (" << localMap[i].x << " , " << localMap[i].y << "), Velocity ("
                 //<< leftSpeed << " , " << rightSpeed << ") "<< endl;

            searchSpace[i].first = leftSpeed;
            searchSpace[i].second = rightSpeed;
        }
    }

    return searchSpace;
}

int ObstacleAvoidance::calculateBestVelocity(QVector<QPair< double,double> > searchSpace)
{
    double distance = 0;
    double targetHeading = 0;
    double clearance = 0;
    int bestValue = 0;
    double bestCost = 0;

    for(int counter = 0; counter < SEARCH_SPACE_GRANULARITY;counter++)
    {
        if (searchSpace[counter].first == 0 && searchSpace[counter].second == 0)
               continue;

        //ldbg << "Obstacle Avoidance - dynamic: Actual Frontier is (" << actualFrontier->getX() << ", " << actualFrontier->getY() <<", " << actualFrontier->getTheta() << ") " << endl;
        Pose predictedPose = forwardKinematics(actualState->getPose(),searchSpace[counter].first, searchSpace[counter].second,DYNAMIC_DELTA_T);

        distance = predictedPose.getDistance(*actualFrontier);
        targetHeading = angularDistance(actualFrontier->getTheta(),predictedPose.getTheta());
        clearance = searchSpace[counter].first;

        //ldbg << "Obstacle Avoidance - dynamic: Predicted pose (" << predictedPose.getX() << " , " << predictedPose.getY() << ", " << predictedPose.getTheta() << "), Velocity ("
            // << searchSpace[counter].first << " , " << searchSpace[counter].second << ") "<< endl;
        //ldbg << "Parameters: Target Heading= "<< targetHeading << " Distance= " << distance << " Clearance= " << clearance<<endl;
        double cost = distance + clearance - targetHeading;
        //ldbg << "Cost = " << cost << endl;
        if (cost>=bestCost)
        {
            //ldbg << "Is best cost!"<<endl;
            bestCost = cost;
            bestValue = counter;
        }
    }



    return bestValue;
}


void ObstacleAvoidance::handleNeuralSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{

}

void ObstacleAvoidance::handleEmpiricSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    this->actualState = actualState;
    this->actualAction = actualAction;

    actualMovement = (movementStateEnum)getActualMovement(actualState->getLeftSpeed(),actualState->getRightSpeed());

    //ldbg<<"Robot Controller - empiric: Actual movement = "<< actualMovement<<endl;

    if(sonar.getPosition() == SonarData::Front && !(actualMovement == BACK))
        handleFrontSonarData(sonar);
    else if(sonar.getPosition() == SonarData::Back && actualMovement == BACK)
        handleBackSonarData(sonar);

}

void ObstacleAvoidance::handleFrontSonarData(const Data::SonarData &sonar)
{
    emit sigChangeActionStartTimestamp(-1);

    if (sonar.getMinDistance()<THRESHOLD)
        handleFrontObstacle(sonar);
    else
    {
        if (actualAction == NULL && reactiveFrontBehaviorStatus > DEACTIVATED)
        {
            reactiveFrontBehaviorStatus = DEACTIVATED;
            //ldbg <<"Robot controller - empiric: Nothing to do. Restart exploration!" << endl;
            emit sigRestartExploration();
        }
    }
}

int ObstacleAvoidance::getActualMovement(double leftSpeed, double rightSpeed)
{
    if (actualAction == NULL)
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
        int type = actualAction->getType();
        double value = actualAction->getValue();

        //ldbg <<"Robot Controller: Next action to do is a " << type << " with a value " << value<<endl;

        if (type == Action::Rotation && value>0)
        {
            //ldbg <<"Robot Controller: I'm rotating to right!"<<endl;
            return RIGHT;
        }
        else if (type == Action::Rotation && value<0)
        {
            //ldbg <<"Robot Controller: I'm rotating to left!"<<endl;
            return LEFT;
        }
        else if (type == Action::Translation && value<0)
        {
            //ldbg <<"Robot Controller: I'm moving backward!"<<endl;
            return BACK;
        }
        else
        {
            //ldbg <<"Robot Controller: I'm going straight" <<endl;
            return FRONT;
        }
    }
}


void ObstacleAvoidance::handleFrontObstacle(const Data::SonarData &sonar)
{
    double distanceLeftL = sonar.getFront(0);
    double distanceLeft = sonar.getFront(1);
    double distanceFront = sonar.getFront(2);
    double distanceRight = sonar.getFront(3);
    double distanceRightR = sonar.getFront(4);

    if (reactiveFrontBehaviorStatus == DEACTIVATED)
    {
        emit sigStopRobot(true);
        //ldbg <<"RobotController - Empiric FSM: DEACTIVATED to FIRSTTIME."<<endl;
        reactiveFrontBehaviorStatus = FIRSTTIME;
    }

    if (reactiveFrontBehaviorStatus > DEACTIVATED)
    {
        emit sigUpdateSonarData(sonar);

        obstacleAvoidanceEmpiricHandler(distanceRightR, distanceLeftL, distanceLeft, distanceFront, distanceRight);
    }

    emit sigChangeRobotControlType(NORMAL);
}

void ObstacleAvoidance::obstacleAvoidanceEmpiricHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL)
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
                            //ldbg <<"Caso LL_L_F_R_RR: Ostacoli ovunque"<<endl;
                            emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
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
                            //ldbg <<"Caso LL_L_F_R: Estrema destra libera"<<endl;
                            emit sigMoveRobot(0,VAI_INDIETRO,0);
                            emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
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
                        //ldbg <<"Caso LL_L_F: Destra libera"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,0);
                        emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
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
                    //ldbg <<"Caso LL_L: Sinistra occupata"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
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
                //ldbg <<"Caso LL: Sinistra occupata"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
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
                        //ldbg <<"Caso L_F_R_RR: Possibile pertugio estrema sinistra"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,0);
                        emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
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
                        //ldbg <<"Caso L_F_R: Estremi liberi. Vado indietro"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
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
                    //ldbg <<"Caso L_F: Destra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
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
                //ldbg <<"Caso L: Sinistra occupata"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
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
                    //ldbg <<"Caso F_R_RR: Sinistra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
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
                    //ldbg <<"Caso F_R: Sinistra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
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
                //ldbg <<"Caso F: Caso indecisione. Vado indietro."<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MIN);
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
                //ldbg <<"Caso R_RR: Sinistra libera"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
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
                //ldbg <<"Caso R: Sinistra libera"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
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
            //ldbg <<"Caso_RR: Sinistra libera"<<endl;
            emit sigMoveRobot(0,VAI_INDIETRO,0);
            emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
            typeMovement = RR;
            //ldbg<< typeMovement <<endl;
        }
        else if(reactiveFrontBehaviorStatus == EXEC && typeMovement!=RR)
        {
            //ldbg << "Typemovement was RR. Now is" << typeMovement <<endl;
            reactiveFrontBehaviorStatus = DEACTIVATED;
        }
    }
    //else
    //ldbg<<"Waiting movement typeMovement" << typeMovement << endl;
}

void ObstacleAvoidance::handleBackObstacle(const Data::SonarData &sonar)
{

    if (reactiveBackBehaviorStatus == DEACTIVATED)
    {
        emit sigStopRobot(true);
        //ldbg <<"RobotController - empiric: DEACTIVATED to FIRSTTIME (back)"<<endl;
        reactiveBackBehaviorStatus = FIRSTTIME;
    }

    if (reactiveBackBehaviorStatus == FIRSTTIME)
    {
        reactiveBackBehaviorStatus = EXEC;
        emit sigMoveRobot(0,VAI_AVANTI,0);
    }
}

void ObstacleAvoidance::handleBackSonarData(const Data::SonarData &sonar)
{
    if(sonar.getMinBackDistance()<THRESHOLD)
        handleBackObstacle(sonar);
    else if (reactiveBackBehaviorStatus> DEACTIVATED && actualAction == NULL)
    {
        emit sigRestartExploration();
    }
}

Data::Pose ObstacleAvoidance::forwardKinematics(const Data::Pose &from, double vr, double vl, double time)
{
    const double dt = time, L = WHEEL_BASE;
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

