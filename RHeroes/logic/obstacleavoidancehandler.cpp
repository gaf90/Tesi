#include "obstacleavoidancehandler.h"
#include "wheelspeeds.h"
#include <data/robotstate.h>
#include <slam/geometry/point.h>
#include <slam/slammodule.h>
#include <shared/constants.h>
#include <shared/utilities.h>
#include <libraries/fann/fann.h>

#define TRAINING_FILE "FANN_training.data"
#define TRAINING_NET "FANN_par.net"



using namespace Data;
using namespace SLAM::Geometry;

QString typeMovementEnumString[16] = {"LLLFRRR","LLLFR","LLLF","LLL","LL","LFRRR","LFR","LF","L","FRRR","FR","F","RRR","RR","R","S"};
QString statusEnumString[3] = {"DEACTIVATED","FIRSTTIME","EXEC"};

ObstacleAvoidance::ObstacleAvoidance(InverseKinematic *inverseKinematic, QObject *parent) : QObject(parent)
{  
    empiricFrontStatus = DEACTIVATED;
    empiricBackStatus = DEACTIVATED;
    neuralBehaviorStatus = DEACTIVATED;
    dwaBehaviorStatus = DEACTIVATED;
    predictedMovement = S;
    oldPredictedMovement = S;
    this->inverseKinematicModule = inverseKinematic;
    algorithmType = DWA;
    if (algorithmType == NEURAL)
    {
        handleNeuralNetwork();
    }
}

void ObstacleAvoidance::handleNeuralNetwork()
{
    QFile training_net(TRAINING_NET);
    if (!training_net.exists())
    {
        ldbg << "FANN: No previous neural network saved."<<endl;
        QFile training_data(TRAINING_FILE);
        if (!training_data.exists())
            ldbg << "FANN: No training data file" << endl;
        else
        {
            num_input = 10;
            num_output = 1;
            num_layers = 3;
            num_neurons_hidden = 110;
            desired_error = (const float) 0.5;
            max_epochs = 50000;
            epochs_between_reports = 2000;

            neuralNetwork = fann_create_standard(num_layers, num_input,
                                                 num_neurons_hidden, num_output);

            fann_set_training_algorithm(neuralNetwork, FANN_TRAIN_RPROP);

            fann_set_activation_function_hidden(neuralNetwork, FANN_ELLIOT);
            fann_set_activation_function_output(neuralNetwork, FANN_LINEAR);

            fann_train_on_file(neuralNetwork, TRAINING_FILE, max_epochs,
                               epochs_between_reports, desired_error);

            fann_save(neuralNetwork, TRAINING_NET);


        }
    }
    else
    {
        neuralNetwork = fann_create_from_file(TRAINING_NET);
    }
}

void ObstacleAvoidance::setSlamModule(SLAM::SLAMModule *slam)
{
    this->slam = slam;
}

void ObstacleAvoidance::setMovementType(int type)
{
    this->sensorDataCaptured = (sensorDataEnum)type;
}

void ObstacleAvoidance::handleObstacle(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    checkSonarData(sonar);
    if (algorithmType == EMPIRIC)
        handleEmpiricSonarData(sonar,actualState,actualAction,actualFrontier);
    else if (algorithmType == DWA)
        handleDynamicWindowSonarData(sonar,actualState,actualAction,actualFrontier);
    else
        handleNeuralSonarData(sonar,actualState,actualAction,actualFrontier);
}

void ObstacleAvoidance::checkSonarData(const Data::SonarData &sonar)
{
    isFrontObstacle = sonar.getPosition() == SonarData::Front && (sonar.getFront(0) < THRESHOLD || sonar.getFront(1) < THRESHOLD || sonar.getFront(2) < THRESHOLD ||sonar.getFront(3) < THRESHOLD ||sonar.getFront(4) < THRESHOLD);
    isBackObstacle =  sonar.getPosition() == SonarData::Back && (sonar.getBack(0) < THRESHOLD || sonar.getBack(1) < THRESHOLD || sonar.getBack(2) < THRESHOLD || sonar.getBack(3) < THRESHOLD || sonar.getBack(4) < THRESHOLD);
}

Data::Pose ObstacleAvoidance::forwardKinematics(const Data::Pose &from, double vr, double vl, double time)
{
    const double dt = time, L = WHEEL_BASE;
    double x, y, theta;
    if(almostEqual(vl, vr)) {
        x = from.x() - vr * dt * sin(from.theta());
        y = from.y() + vr * dt * cos(from.theta());
        theta = from.theta();
    } else {
        const double R = .5 * L * (vr + vl) / (vr - vl);
        const double w = (vr - vl) / L;
        x = from.x() + R * cos(from.theta() + w * dt) - R * cos(from.theta());
        y = from.y() + R * sin(from.theta() + w * dt) - R * sin(from.theta());
        theta = wrapRad(from.theta() + w * dt);
    }
    return Data::Pose(x, y, theta);
}



//Empiric
void ObstacleAvoidance::handleEmpiricSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    this->actualState = actualState;
    this->actualAction = actualAction;

    actualMovement = (movementStateEnum)getActualMovement(actualState->getLeftSpeed(),actualState->getRightSpeed());


    ldbg<<"EMPIRIC: Actual movement = "<< actualMovement<<endl;

    emit sigChangeActionStartTimestamp(-1);

    if(actualMovement == BACK)
    {
        if(isBackObstacle)
        {
            if (empiricBackStatus == DEACTIVATED)
            {
                emit sigStopRobot(true);
                ldbg <<"EMPIRIC (Back): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
                empiricBackStatus = FIRSTTIME;
            }

            else if (empiricBackStatus == FIRSTTIME)
            {
                emit sigUpdateSonarData(sonar);
                empiricBackStatus = EXEC;
                emit sigMoveRobot(0,VAI_AVANTI,0);
            }

            emit sigChangeRobotControlType(NORMAL);
        }
        else if (empiricBackStatus == EXEC && actualAction == NULL)
        {
            empiricBackStatus = DEACTIVATED;
            emit sigRestartExploration();
        }
    }
    else
    {
        if (isFrontObstacle)
        {
            if (empiricFrontStatus == DEACTIVATED)
            {
                emit sigStopRobot(true);
                ldbg <<"EMPIRIC (Front): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
                empiricFrontStatus = FIRSTTIME;
            }

            else if (empiricFrontStatus > DEACTIVATED)
            {
                emit sigUpdateSonarData(sonar);
                empiricObstacleHandler(sonar.getFront(4), sonar.getFront(0), sonar.getFront(1), sonar.getFront(2), sonar.getFront(3));
            }

            emit sigChangeRobotControlType(NORMAL);
        }
        else if (empiricFrontStatus > DEACTIVATED && actualAction == NULL)
        {
            empiricFrontStatus = DEACTIVATED;
            ldbg <<"EMPIRIC: Nothing to do. Restart exploration!" << endl;
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

        ldbg <<"Robot Controller: Next action to do is a " << type << " with a value " << value<<endl;

        if (type == Action::Rotation && value>0)
            return RIGHT;
        else if (type == Action::Rotation && value<0)
            return LEFT;
        else if (type == Action::Translation && value<0)
            return BACK;
        else
            return FRONT;
    }
}

void ObstacleAvoidance::empiricObstacleHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL)
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
                        if (empiricFrontStatus == FIRSTTIME)
                        {
                            empiricFrontStatus = EXEC;
                            ////ldbg <<"Caso LL_L_F_R_RR: Ostacoli ovunque"<<endl;
                            emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                            sensorDataCaptured = LLLFRRR;
                            ////ldbg<< typeMovement <<endl;
                        }
                        else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LLLFRRR)
                        {
                            ////ldbg << "Typemovement was LLLFRRR. Now is" << typeMovement <<endl;
                            empiricFrontStatus = DEACTIVATED;
                        }
                    }
                    else
                    {
                        if (empiricFrontStatus == FIRSTTIME)
                        {
                            empiricFrontStatus = EXEC;
                            ////ldbg <<"Caso LL_L_F_R: Estrema destra libera"<<endl;
                            emit sigMoveRobot(0,VAI_INDIETRO,0);
                            emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                            sensorDataCaptured = LLLFR;
                            ////ldbg<< typeMovement <<endl;
                        }
                        else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LLLFR)
                        {
                            ////ldbg << "Typemovement was LLLFR. Now is" << typeMovement <<endl;
                            empiricFrontStatus = DEACTIVATED;
                        }
                    }
                }
                else
                {
                    if (empiricFrontStatus == FIRSTTIME)
                    {
                        empiricFrontStatus = EXEC;
                        ldbg <<"Caso LL_L_F: Destra libera"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,0);
                        emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                        sensorDataCaptured = LLLF;
                    }
                    else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LLLF)
                    {
                        ////ldbg << "Typemovement was LLLF. Now is" << typeMovement <<endl;
                        empiricFrontStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (empiricFrontStatus == FIRSTTIME)
                {
                    empiricFrontStatus = EXEC;
                    ldbg <<"Caso LL_L: Sinistra occupata"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                    sensorDataCaptured = LLL;
                    ////ldbg<< typeMovement <<endl;
                }
                else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LLL)
                {
                    ////ldbg << "Typemovement was LLL. Now is" << typeMovement <<endl;
                    empiricFrontStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (empiricFrontStatus == FIRSTTIME)
            {
                empiricFrontStatus = EXEC;
                ldbg <<"Caso LL: Sinistra occupata"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                sensorDataCaptured = LL;
                ////ldbg<< typeMovement <<endl;
            }
            else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LL)
            {
                ////ldbg << "Typemovement was LL. Now is" << typeMovement <<endl;
                empiricFrontStatus = DEACTIVATED;
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
                    if (empiricFrontStatus == FIRSTTIME)
                    {
                        empiricFrontStatus = EXEC;
                        ldbg <<"Caso L_F_R_RR: Possibile pertugio estrema sinistra"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,0);
                        emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                        sensorDataCaptured = LFRRR;
                        ////ldbg<< typeMovement <<endl;
                    }
                    else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LFRRR)
                    {
                        ////ldbg << "Typemovement was LFRR. Now is" << typeMovement <<endl;
                        empiricFrontStatus = DEACTIVATED;
                    }
                }
                else
                {
                    if (empiricFrontStatus == FIRSTTIME)
                    {
                        empiricFrontStatus = EXEC;
                        ldbg <<"Caso L_F_R: Estremi liberi. Vado indietro"<<endl;
                        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                        sensorDataCaptured = LFR;
                        ////ldbg<< typeMovement <<endl;
                    }
                    else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LFR)
                    {
                        ////ldbg << "Typemovement was LFR. Now is" << typeMovement <<endl;
                        empiricFrontStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (empiricFrontStatus == FIRSTTIME)
                {
                    empiricFrontStatus = EXEC;
                    ldbg <<"Caso L_F: Destra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                    sensorDataCaptured = LF;
                    ////ldbg<< typeMovement <<endl;
                }
                else if(empiricFrontStatus == EXEC && sensorDataCaptured!=LF)
                {
                    ////ldbg << "Typemovement was LF. Now is" << typeMovement <<endl;
                    empiricFrontStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (empiricFrontStatus == FIRSTTIME)
            {
                empiricFrontStatus = EXEC;
                ldbg <<"Caso L: Sinistra occupata"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                sensorDataCaptured = L;
                ////ldbg<< typeMovement <<endl;
            }
            else if(empiricFrontStatus == EXEC && sensorDataCaptured!=L)
            {
                ////ldbg << "Typemovement was L. Now is" << typeMovement <<endl;
                empiricFrontStatus = DEACTIVATED;
            }

        }
    }
    else if (distanceFront < THRESHOLD)
    {
        if (distanceRight < THRESHOLD)
        {
            if (distanceRightR < THRESHOLD)
            {
                if (empiricFrontStatus == FIRSTTIME)
                {
                    empiricFrontStatus = EXEC;
                    ldbg <<"Caso F_R_RR: Sinistra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    sensorDataCaptured = FRRR;
                    ////ldbg<< typeMovement <<endl;
                }
                else if(empiricFrontStatus == EXEC && sensorDataCaptured!=FRRR)
                {
                    ////ldbg << "Typemovement was FRR. Now is" << typeMovement <<endl;
                    empiricFrontStatus = DEACTIVATED;
                }
            }
            else
            {
                if (empiricFrontStatus == FIRSTTIME)
                {
                    empiricFrontStatus = EXEC;
                    ldbg <<"Caso F_R: Sinistra libera"<<endl;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);
                    emit sigMoveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    sensorDataCaptured = FR;
                    ////ldbg<< typeMovement <<endl;
                }
                else if(empiricFrontStatus == EXEC && sensorDataCaptured!=FR)
                {
                    ////ldbg << "Typemovement was FR. Now is" << typeMovement <<endl;
                    empiricFrontStatus = DEACTIVATED;
                }
            }
        }
        else
        {

            if (empiricFrontStatus == FIRSTTIME)
            {
                empiricFrontStatus = EXEC;
                ldbg <<"Caso F: Caso indecisione. Vado indietro."<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MIN);
                sensorDataCaptured = F;
                ////ldbg<< typeMovement <<endl;
            }
            else if(empiricFrontStatus == EXEC && sensorDataCaptured!=F)
            {
                ////ldbg << "Typemovement was F. Now is" << typeMovement <<endl;
                empiricFrontStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceRight < THRESHOLD)
    {
        if (distanceRightR < THRESHOLD)
        {
            if (empiricFrontStatus == FIRSTTIME)
            {
                empiricFrontStatus = EXEC;
                ////ldbg <<"Caso R_RR: Sinistra libera"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                sensorDataCaptured = RRR;
                ////ldbg<< typeMovement <<endl;
            }
            else if(empiricFrontStatus == EXEC && sensorDataCaptured!=RRR)
            {
                ////ldbg << "Typemovement was RRR. Now is" << typeMovement <<endl;
                empiricFrontStatus = DEACTIVATED;
            }
        }
        else
        {
            if (empiricFrontStatus == FIRSTTIME)
            {
                empiricFrontStatus = EXEC;
                ldbg <<"Caso R: Sinistra libera"<<endl;
                emit sigMoveRobot(0,VAI_INDIETRO,0);
                emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                sensorDataCaptured = R;
                ////ldbg<< typeMovement <<endl;
            }
            else if(empiricFrontStatus == EXEC  && sensorDataCaptured!=R)
            {
                ////ldbg << "Typemovement was R. Now is" << typeMovement <<endl;
                empiricFrontStatus = DEACTIVATED;;
            }
        }
    }
    else if (distanceRightR < THRESHOLD)
    {
        if (empiricFrontStatus == FIRSTTIME)
        {
            empiricFrontStatus = EXEC;
            ldbg <<"Caso_RR: Sinistra libera"<<endl;
            emit sigMoveRobot(0,VAI_INDIETRO,0);
            emit sigMoveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
            sensorDataCaptured = RR;
            ////ldbg<< typeMovement <<endl;
        }
        else if(empiricFrontStatus == EXEC && sensorDataCaptured!=RR)
        {
            ////ldbg << "Typemovement was RR. Now is" << typeMovement <<endl;
            empiricFrontStatus = DEACTIVATED;
        }
    }
}


//DWA

void ObstacleAvoidance::handleDynamicWindowSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier)
{

    this->actualAction = actualAction;
    this->actualFrontier = actualFrontier;
    this->actualState = actualState;

    double leftSpeed = actualState->getLeftSpeed();
    double rightSpeed = actualState->getRightSpeed();

    emit sigChangeActionStartTimestamp(-1);

    if (leftSpeed !=0 && rightSpeed!=0)
    {

        Pose actualPose = actualState->getPose();
        Pose predictedPose = forwardKinematics(actualPose,leftSpeed, rightSpeed,DYNAMIC_DELTA_T);

        ldbg<<"DWA: Predicted pose is (" << predictedPose.getX() << "," << predictedPose.getY() <<
              "," << predictedPose.getTheta() << "), behavior status is: " << statusEnumString[dwaBehaviorStatus] << ", sonar obstacle detection " << isFrontObstacle << endl;


        if (!isReachablePose(predictedPose, actualPose) || isFrontObstacle)
        {
            if (dwaBehaviorStatus == DEACTIVATED)
            {
                emit sigStopRobot(true);
                dwaBehaviorStatus = FIRSTTIME;
            }

            if (dwaBehaviorStatus == FIRSTTIME)
            {
                emit sigUpdateSonarData(sonar);

                ldbg <<"DWA: Start. Actual Pose is ("<<
                       actualPose.getX() << ", " << actualPose.getY() <<", " << actualPose.getTheta() <<")"<<endl;

                QVector<QPair< double,double> > searchSpace = calculateSearchSpace(sonar);

                int bestValue = calculateBestVelocity(searchSpace);
                if (bestValue == -1)
                {
                    ldbg <<"DWA: No position found." <<endl;
                    dwaBehaviorStatus = EXEC;
                    emit sigMoveRobot(0,VAI_INDIETRO,0);

                }
                else
                {
                    Pose bestPose = forwardKinematics(actualPose,searchSpace[bestValue].first,searchSpace[bestValue].second,DYNAMIC_DELTA_T);
                    ldbg <<"DWA: Best pose is (" << bestPose.getX() << ", " << bestPose.getY() <<  ", " << bestPose.getTheta() << ") " << endl;
                    double trasl = bestPose.getDistance(actualPose);
                    double rot = computeRotationFromPoses(actualPose,bestPose);

                    ldbg <<"DWA: I must rotate of " << rot << endl;
                    ldbg <<"DWA: I must traslate of " << trasl << endl;

                    dwaBehaviorStatus = EXEC;
                    emit sigMoveRobot(rot, trasl,0);
                }
                emit sigChangeRobotControlType(NORMAL);
            }
        }
    }
    else
    {
        if (dwaBehaviorStatus == EXEC)
            dwaBehaviorStatus = DEACTIVATED;
        if (actualAction == NULL)
        {
            ldbg <<"DWA: Nothing to do. Restart exploration!" << endl;
            emit sigRestartExploration();
        }
    }
}


bool ObstacleAvoidance::isReachablePose(Pose predictedPose, Pose actualPose)
{
    Point* predictedPosePoint = new Point(predictedPose.getX(), predictedPose.getY());
    Point* actualPosePoint = new Point(actualPose.getX(),actualPose.getY());

    bool isReachable = slam->getMap(true).isReachable(*actualPosePoint,*predictedPosePoint,P3AT_RADIUS);

    return isReachable;
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
            ////ldbg << "Angle = "<<angle << " Cos = " << cos(angle) << " Sen = " << sin(angle)
            //    << "Actual X = " << actualX << "Actual Y = " << actualY;

            double deltaX = actualX + step*radius*cos(angle);
            double deltaY = actualY + step*radius*sin(angle);

            Pose deltaPose(deltaX,deltaY,angle);

            if (isReachablePose(deltaPose,actualPose))
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

    //Print local map
    for (double i=0; i<SEARCH_SPACE_GRANULARITY; i++)
        ldbg << "DWA: Angle = "<< localMap[i].angle << ", X = " << localMap[i].x
             << " , Y = " << localMap[i].y << ", Distance = " << localMap[i].distance << endl;


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
            double dXv = (localMap[i].x - actualState->getPose().getX())*cos(actualState->getPose().getTheta()) -
                    (localMap[i].y - actualState->getPose().getY())*sin(actualState->getPose().getTheta());

            double curvature = 2*dXv/localMap[i].distance;

            double leftSpeed = (MED_SPEED/RH_RADIUS)*(1 - (WHEEL_BASE/2)*curvature);
            double rightSpeed = (MED_SPEED/RH_RADIUS)*(1 + (WHEEL_BASE/2)*curvature);

            ldbg << "DWA: Pose (" << localMap[i].x << " , " << localMap[i].y << "), Velocity ("
                 << leftSpeed << " , " << rightSpeed << ") "<< endl;

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

        ldbg << "DWA: Actual Frontier is (" << actualFrontier->getX() << ", " << actualFrontier->getY() <<", " << actualFrontier->getTheta() << ") " << endl;
        Pose predictedPose = forwardKinematics(actualState->getPose(),searchSpace[counter].first, searchSpace[counter].second,DYNAMIC_DELTA_T);

        distance = predictedPose.getDistance(*actualFrontier);
        targetHeading = angularDistance(actualFrontier->getTheta(),predictedPose.getTheta());
        clearance = searchSpace[counter].first;

        ldbg << "DWA: Predicted pose (" << predictedPose.getX() << " , " << predictedPose.getY() << ", " << predictedPose.getTheta() << "), Velocity ("
             << searchSpace[counter].first << " , " << searchSpace[counter].second << ") "<< endl;
        ldbg << "Parameters: Target Heading= "<< targetHeading << " Distance= " << distance << " Clearance= " << clearance<<endl;
        double cost = distance + clearance - targetHeading;
        ldbg << "Cost = " << cost << endl;
        if (cost>=bestCost)
        {
            ldbg << "DWA: Is best cost!"<<endl;
            bestCost = cost;
            bestValue = counter;
        }
    }



    return bestValue;
}

//Neural
void ObstacleAvoidance::handleNeuralSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    this->actualState = actualState;
    this->actualAction = actualAction;
    this->actualState = actualState;

    emit sigChangeActionStartTimestamp(-1);

    if (isFrontObstacle || isBackObstacle)
    {
        ldbg << "FANN: Neural Behavior Status is "<< statusEnumString[neuralBehaviorStatus]<<endl;
        ldbg << "FANN: Minimum frontal distance is " << sonar.getMinDistance() << endl;
        if (neuralBehaviorStatus == DEACTIVATED)
        {
            ldbg << "FANN: Stop robot. " << endl;
            emit sigStopRobot(true);
            neuralBehaviorStatus = FIRSTTIME;
        }
        else if (neuralBehaviorStatus > DEACTIVATED)
        {
            emit sigUpdateSonarData(sonar);

            fann_type input[10] = {sonar.getFront(0), sonar.getFront(1), sonar.getFront(2), sonar.getFront(3), sonar.getFront(4),
                                   sonar.getBack(0),sonar.getBack(1),sonar.getBack(2),sonar.getBack(3),sonar.getBack(4)};

            ldbg <<"FANN: Input file is " << input << endl;
            oldPredictedMovement = predictedMovement;
            predictedMovement = (int)(fann_run(neuralNetwork,input)[0]);

            ldbg << "FANN: Predicted action is " << predictedMovement
                 <<". Previous movement is " << oldPredictedMovement << endl;
            if (predictedMovement != oldPredictedMovement)
            {
                neuralBehaviorStatus = EXEC;
                ldbg << "FANN: Need to change movement. Apply predicted action." << endl;
                applyPredictedAction(predictedMovement);
            }
            else
                ldbg <<"FANN: Already do that movement. Ignore sonar data" << endl;
        }

        emit sigChangeRobotControlType(NORMAL);
    }
    else if (actualAction == NULL && neuralBehaviorStatus > DEACTIVATED)
    {
        ldbg << "FANN: Restart exploration."<< endl;
        neuralBehaviorStatus = DEACTIVATED;
        predictedMovement = S;
        oldPredictedMovement = S;
        emit sigStopRobot(true);
        emit sigRestartExploration();
    }

}

void ObstacleAvoidance::applyPredictedAction(int predictedMovement)
{    
    switch (predictedMovement)
    {
    case LLLFRRR:
    {
        ldbg <<"FANN: Go back and rotate to right (max)." << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
        break;
    }
    case LLLFR:
    {
        ldbg <<"FANN: Go back and rotate to right (max)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
        break;
    }
    case LLLF:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MED);
        break;
    }
    case LLL:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MED);
        break;
    }
    case LL:
    {
        ldbg <<"FANN: Go back and rotate to right (min)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MIN);
        break;
    }
    case LFRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MAX);
        break;
    }
    case LFR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MAX);
        break;
    }
    case LF:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MED);
        break;
    }
    case L:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTADESTRA_MED);
        break;
    }
    case FRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MED);
        break;
    }
    case FR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MED);
        break;
    }
    case F:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MED);
        break;
    }
    case RRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MAX);
        break;
    }
    case RR:
    {
        ldbg <<"FANN: Go back and rotate to left (min)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MIN);
        break;
    }
    case R:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,VAI_INDIETRO,RUOTASINISTRA_MED);
        break;
    }
    default:
    {
        ldbg <<"FANN: Prediction error."<<endl;
        break;
    }
    }
    emit sigMoveRobot(0,VAI_AVANTI,0);
}




