#include "obstacleavoidancehandler.h"
#include "wheelspeeds.h"
#include <data/robotstate.h>
#include <data/laserdata.h>
#include <slam/geometry/point.h>
#include <slam/slammodule.h>
#include <shared/constants.h>
#include <shared/utilities.h>
#include <libraries/fann/fann.h>
#include <exception>


#define TRAINING_FILE "FANN_training.data"
#define TRAINING_NET "FANN_par.net"



using namespace Data;
using namespace SLAM::Geometry;
using namespace std;

QString typeMovementEnumString[16] = {"LLLFRRR","LLLFR","LLLF","LLL","LL","LFRRR","LFR","LF","L","FRRR","FR","F","RRR","RR","R","S"};
QString statusEnumString[3] = {"DEACTIVATED","FIRSTTIME","EXEC"};

ObstacleAvoidance::ObstacleAvoidance(InverseKinematic *inverseKinematic, int angleTol, QObject *parent) : QObject(parent)
{  
    empiricFrontStatus = DEACTIVATED;
    empiricBackStatus = DEACTIVATED;
    neuralBehaviorStatus = DEACTIVATED;
    dwaBehaviorStatus = DEACTIVATED;
    predictedMovement = S;
    oldPredictedMovement = S;
    this->angleTol = angleTol;
    this->inverseKinematicModule = inverseKinematic;
    algorithmType = NEURAL;
    if (algorithmType == NEURAL)
    {
        handleNeuralNetwork();
    }
    isLaser = false;
    bestX = 0;
    bestY = 0;
    bestTheta = 0;
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

void ObstacleAvoidance::handleObstacle(const Data::SonarData &sonar, Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
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
    isSonarObstacle = sonar.getPosition() == SonarData::Front && (sonar.getFront(0) < THRESHOLD || sonar.getFront(1) < THRESHOLD || sonar.getFront(2) < THRESHOLD ||sonar.getFront(3) < THRESHOLD ||sonar.getFront(4) < THRESHOLD);
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
                emit sigMoveRobot(0,GO_STRAIGHT,0);
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
        if (isSonarObstacle)
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
        if (type == Action::Rotation)
            value = fromRadiantToDegree(value);

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
                            emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MAX);
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
                            emit sigMoveRobot(0,GO_BACK,0);
                            emit sigMoveRobot(ROTATERIGHT_MAX,GO_STRAIGHT,0);
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
                        emit sigMoveRobot(0,GO_BACK,0);
                        emit sigMoveRobot(ROTATERIGHT_MAX,GO_STRAIGHT,0);
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
                    emit sigMoveRobot(0,GO_BACK,0);
                    emit sigMoveRobot(ROTATERIGHT_MED,GO_STRAIGHT,0);
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
                emit sigMoveRobot(0,GO_BACK,0);
                emit sigMoveRobot(ROTATERIGHT_MED,GO_STRAIGHT,0);
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
                        emit sigMoveRobot(0,GO_BACK,0);
                        emit sigMoveRobot(ROTATELEFT_MAX,GO_STRAIGHT,0);
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
                        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MAX);
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
                    emit sigMoveRobot(0,GO_BACK,0);
                    emit sigMoveRobot(ROTATERIGHT_MAX,GO_STRAIGHT,0);
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
                emit sigMoveRobot(0,GO_BACK,0);
                emit sigMoveRobot(ROTATERIGHT_MED,GO_STRAIGHT,0);
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
                    emit sigMoveRobot(0,GO_BACK,0);
                    emit sigMoveRobot(ROTATELEFT_MAX,GO_STRAIGHT,0);
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
                    emit sigMoveRobot(0,GO_BACK,0);
                    emit sigMoveRobot(ROTATELEFT_MAX,GO_STRAIGHT,0);
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
                emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MIN);
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
                emit sigMoveRobot(0,GO_BACK,0);
                emit sigMoveRobot(ROTATELEFT_MIN,GO_STRAIGHT,0);
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
                emit sigMoveRobot(0,GO_BACK,0);
                emit sigMoveRobot(ROTATELEFT_MED,GO_STRAIGHT,0);
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
            emit sigMoveRobot(0,GO_BACK,0);
            emit sigMoveRobot(ROTATELEFT_MIN,GO_STRAIGHT,0);
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
    this->actualPose = actualState->getPose();

    bool isLaserObstacle = false;

    emit sigChangeActionStartTimestamp(-1);

    isLaserObstacle = checkLaserData();

    //ldbg <<"DWA: State is "<< statusEnumString[dwaBehaviorStatus]<< ". Laser obstaccle " << isLaserObstacle << ". Sonar obstacle " << isSonarObstacle << endl;

    if (isLaserObstacle)
    {
        if (dwaBehaviorStatus == DEACTIVATED)
        {
            emit sigStopRobot(true);
            dwaBehaviorStatus = FIRSTTIME;
            ldbg <<"DWA: DEACTIVATED to FIRSTIME (Inner)"<< endl;
        }

        if (dwaBehaviorStatus > DEACTIVATED)
        {
            emit sigUpdateSonarData(sonar);
            Pose actualPose = actualState->getPose();

            laserReadings = actualLaser->getReadings();

            calculateSearchSpace(actualPose);

            int bestValue = calculateBestVelocity();

            bestLeftSpeed = searchSpaceVelocities[bestValue].first;
            bestRightSpeed = searchSpaceVelocities[bestValue].second;


            bestX = searchSpacePoses[bestValue].x;
            bestY = searchSpacePoses[bestValue].y;
            bestTheta = searchSpacePoses[bestValue].angle;

            bestPose = Pose(bestX, bestY, bestTheta);

            ldbg << "DWA: Actual frontier is ("<< *actualFrontier<<
                    "). Best pose is (" << bestPose
                 <<"). Velocity is " << bestLeftSpeed << ", " << bestRightSpeed << endl;


            dwaBehaviorStatus = EXEC;

            emit sigDoMovement(bestLeftSpeed , bestRightSpeed);
            //emit sigChangeRobotControlType(NORMAL);
        }
    }
    else
    {
        if (dwaBehaviorStatus> DEACTIVATED)
        {
            double distance = actualPose.getDistance(bestPose);

            ldbg <<"DWA: Actual pose is " << actualPose << endl;
            ldbg <<"DWA: Best pose is "<<bestPose<<endl;
            ldbg <<"DWA: Distance is " << distance <<endl;

            if (distance<=DWA_DISTANCE||distance>0.1)
            {
                dwaBehaviorStatus = DEACTIVATED;
                ldbg <<"DWA: Restart exploration."<< endl;
                emit sigRestartExploration();
            }
        }
    }
}

bool ObstacleAvoidance::checkLaserData()
{
    bool isLaserObstacle = false;
    if (isLaser)
    {
        isLaser = false;
        double min = LASER_THRESHOLD;
        for (int i =0; i<actualLaser->getReadings().size();i++)
        {
            if (actualLaser->getReadings()[i] < min)
            {
                min = actualLaser->getReadings()[i];
                ldbg <<"DWA: Found obstacle at angle " << i << " with distance " << min << endl;
                isLaserObstacle = true;
            }
        }
    }
    return isLaserObstacle;
}

int ObstacleAvoidance::getLaserID(double angle)
{
    int laserID = fromRadiantToDegree(angle);
    if (laserID<0)
        laserID = 360 - laserID;
    if (laserID>360)
        laserID = laserID - 360;

    return laserID;
}

bool ObstacleAvoidance::checkSafePose(int laserID, LocalMapEl actualSample)
{
    for (int safeLaser = laserID - DWA_SAFETY; safeLaser<laserID + DWA_SAFETY; safeLaser++)
    {
        if (safeLaser<0)
            safeLaser = 360 + safeLaser;

        laserDistance = laserReadings.at(safeLaser);
        actualDistance = actualSample.distance;

        //ldbg<<"DWA: Actual distance = "<< actualDistance <<", Laser distance = "<< laserDistance;
        if (actualDistance>laserDistance)
            return false;
    }

    return true;
}

void ObstacleAvoidance::calculateSearchSpace(Pose actualPose)
{
    searchSpacePoses.clear();
    searchSpaceVelocities.clear();

    ldbg << "========= CALCULATE SEARCH SPACE =========" << endl;

    for (double i=-DWA_MAX_VELOCITY; i<=DWA_MAX_VELOCITY; i+=DWA_STEP)
    {
        for (double j=-DWA_MAX_VELOCITY; j<=DWA_MAX_VELOCITY; j+=DWA_STEP)
        {
            LocalMapEl actualSample;

            Pose predictedPose = forwardKinematics(actualPose,i,j,DWA_TIME);
            actualSample.x = predictedPose.getX();
            actualSample.y = predictedPose.getY();
            actualSample.angle = predictedPose.getTheta();
            actualSample.distance = predictedPose.getDistance(actualPose);

            ldbg << "DWA: Actual sample is ("<<i <<", "<< j<<
                "). Actual pose is "<< actualPose << ", predicted pose is "<< predictedPose<<endl;


            int laserID = getLaserID(actualSample.angle);

            safePose = checkSafePose(laserID, actualSample);

            if (safePose)
            {
                ldbg<<"DWA: Safe pose. Append velocities "<< i << ", " << j <<endl;
                QPair<double,double> velocity(i,j);
                searchSpaceVelocities.append(velocity);
                searchSpacePoses.append(actualSample);
            }
        }
    }
}

int ObstacleAvoidance::calculateBestVelocity()
{
    double velocity= 0;
    double targetHeading = 0;
    double clearance = 0;
    int bestValue = 0;
    double bestCost = 10;

    ldbg << "========= CALCULATE BEST VELOCITY =========" << endl;

    ldbg << "DWA: Search space size = " << searchSpaceVelocities.size() << ", Search poses size = " << searchSpacePoses.size() <<
            ", Obstacle size = " << laserReadings.size() << endl;


    for(int counter = 0; counter < searchSpaceVelocities.size(); counter++)
    {
        Pose predictedPose (searchSpacePoses.at(counter).x, searchSpacePoses.at(counter).y, searchSpacePoses.at(counter).angle);

        ldbg << "DWA: Actual Frontier is " <<*actualFrontier;
        ldbg << ". Predicted pose is "<< predictedPose << endl;

        int laserID = getLaserID(predictedPose.getTheta());

        //ldbg << "DWA: Actual laser is " <<laserID <<endl;

        double obstacleDistance = laserReadings.at(laserID);
        double predictedPoseDistance = searchSpacePoses.at(counter).distance;

        if (obstacleDistance<predictedPoseDistance)
            ldbg << "DWA: Error 1(Predicted pose not reachable)"<<endl;

        clearance = obstacleDistance - predictedPoseDistance;
        targetHeading = computeRotationFromPoses(predictedPose,*actualFrontier);
        velocity = (searchSpaceVelocities.at(counter).first + searchSpaceVelocities.at(counter).second)/2;

        ldbg << "Parameters: Counter = "<< counter << " Target = " << targetHeading << ", Clearance = " << clearance << ", Velocity = " << velocity << endl;

        double t = (360 - abs(targetHeading))/360;
        double c = clearance/obstacleDistance;
        double v = abs(velocity);

        ldbg << "Parameters(Normalized) Counter = "<< counter << " Target = " << DWA_TARGET*t << ", Clearance = " << DWA_CLEARANCE*c << ", Velocity = " << DWA_VELOCITY*v << endl;

        double cost = DWA_SIGMA*(DWA_TARGET*t + DWA_CLEARANCE*c + DWA_VELOCITY*v);
        ldbg << "Cost = " << cost << endl;
        if (cost<=bestCost)
        {
            ldbg << "DWA: Is best cost!"<<endl;
            bestCost = cost;
            bestValue = counter;
        }
    }

    ldbg << "Parameters: Target Heading = "<< fromRadiantToDegree(targetHeading) << ", Clearance = " << clearance << ", Velocity= " << velocity<<endl;
    ldbg << "Cost = "<<bestCost <<endl;

    return bestValue;
}

//Neural
void ObstacleAvoidance::handleNeuralSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{
    this->actualState = actualState;
    this->actualAction = actualAction;
    this->actualState = actualState;

    emit sigChangeActionStartTimestamp(-1);

    if (isSonarObstacle || isBackObstacle)
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
    else
    {
        if (neuralBehaviorStatus == EXEC)
        {
            neuralBehaviorStatus = DEACTIVATED;
            predictedMovement = S;
            oldPredictedMovement = S;
        }
        if (actualAction == NULL)
        {
            ldbg << "FANN: Restart exploration."<< endl;
            emit sigStopRobot(true);
            emit sigRestartExploration();
        }
    }

}

void ObstacleAvoidance::applyPredictedAction(int predictedMovement)
{
    switch (predictedMovement)
    {
    case LLLFRRR:
    {
        ldbg <<"FANN: Go back and rotate to right (max)." << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MAX);
        break;
    }
    case LLLFR:
    {
        ldbg <<"FANN: Go back and rotate to right (max)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MAX);
        break;
    }
    case LLLF:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MED);
        break;
    }
    case LLL:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MED);
        break;
    }
    case LL:
    {
        ldbg <<"FANN: Go back and rotate to right (min)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MIN);
        break;
    }
    case LFRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MAX);
        break;
    }
    case LFR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MAX);
        break;
    }
    case LF:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MED);
        break;
    }
    case L:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATERIGHT_MED);
        break;
    }
    case FRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MED);
        break;
    }
    case FR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MED);
        break;
    }
    case F:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MED);
        break;
    }
    case RRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MAX);
        break;
    }
    case RR:
    {
        ldbg <<"FANN: Go back and rotate to left (min)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MIN);
        break;
    }
    case R:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,GO_BACK,ROTATELEFT_MED);
        break;
    }
    default:
    {
        ldbg <<"FANN: Prediction error."<<endl;
        break;
    }
    }
    emit sigMoveRobot(0,GO_STRAIGHT,0);
}

void ObstacleAvoidance::setLaser(Data::LaserData &laser)
{
    isLaser = true;
    actualLaser = new LaserData(laser.getTimestamp(),laser.getFOV(),laser.getResolution(), laser.getReadings());
}






