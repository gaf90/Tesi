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
QString statusEnumString[7] = {"DEACTIVATED","FIRSTTIME","EXEC", "EXEC_F","EXEC_B","EXEC_R","EXEC_L" };
QString actualMovementString[5] = {"FRONT","RIGHT","LEFT","BACK","STOP"};

ObstacleAvoidance::ObstacleAvoidance(InverseKinematic *inverseKinematic, int angleTol, QObject *parent) : QObject(parent)
{  
    empiricBehaviorStatus = DEACTIVATED;
    neuralBehaviorStatus = DEACTIVATED;
    dwaBehaviorStatus = DEACTIVATED;
    predictedMovement = S;
    oldPredictedMovement = S;
    this->angleTol = angleTol;
    this->inverseKinematicModule = inverseKinematic;
    //ldbg << "OBS: Type "<< (obstacleAlgEnum)Config::OBS::obstacle_algorithm << endl;
    algorithmType = (obstacleAlgEnum)Config::OBS::obstacle_algorithm;
    isLaserObstacleDWA = false;
    if (algorithmType == NEURAL)
    {
        handleNeuralNetwork();
    }

    isLaser = false;
    bestPose = Pose(0,0,0);
    previousActualPose = Pose(0,0,0);
    meters = 0;
    previousRotation = S;
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
    this->previousRotation = (sensorDataEnum) type;
}

void ObstacleAvoidance::handleMetrics()
{
    double isMovingDistance = previousActualPose.getDistance(actualPose);;
    if (isMovingDistance>0.01)
    {
        ldbg <<"Obstacle Metric: ActualMovement is  "<<actualMovementString[actualMovement] << ". Actual Pose is" << actualPose <<". Past pose is " << pastState->getPose() <<endl;
        ldbg << "Actual frontier is "<<*actualFrontier << ". Local is " << localFrontier<< endl;

        double deltaSpace = actualPose.getDistance(pastState->getPose());
        meters+= deltaSpace;
        ldbg <<"Obstacle Metric: Delta space is "<< deltaSpace << ". Total meters is "<<meters;

        double distance = actualPose.getDistance(*actualFrontier);
        ldbg <<". Frontier distance is " << distance << endl;

        if (distance<0.2)
        {
            ldbg << "Obstacle Metric: Reached frontier."<<endl;
            meters = 0;
            emit sigFrontierReached();

        }
    }
}

void ObstacleAvoidance::calcLocalFrontier(Data::Pose *actualFrontier)
{
    double fx = actualFrontier->getX()-actualPose.getX();
    double fy = actualFrontier->getY()-actualPose.getY();
    double rotationAngle = -actualPose.getTheta();

    double fxRot = fx*cos(rotationAngle)-fy*sin(rotationAngle);
    double fyRot = fx*sin(rotationAngle)+fy*cos(rotationAngle);
    double fThetaRot = atan2(fyRot,fxRot);

    Pose localFrontier(fxRot, fyRot, fThetaRot);
    this->localFrontier = localFrontier;
    this->frontierAngle = fromRadiantToDegree(fThetaRot);
    ldbg<<"Obstacle Metrics: Frontier angle is " << frontierAngle<<endl;
}

void ObstacleAvoidance::handleObstacle(const Data::SonarData &sonar, Data::RobotState *pastState, Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose*actualFrontier)
{

    this->actualState = actualState;
    this->actualAction = actualAction;
    this->actualFrontier = actualFrontier;
    this->previousActualPose = actualPose;
    this->actualPose = actualState->getPose();
    this->pastState = pastState;

    actualMovement = (movementStateEnum)getActualMovement(actualState->getLeftSpeed(),actualState->getRightSpeed());

    checkSonarData(sonar);
    //ldbg << "New sonar coming at "<< QTime::currentTime().toString()<<endl;

    calcLocalFrontier(actualFrontier);

    emit sigHandleTimer();
    emit sigUpdateSonarData(sonar);
    emit sigChangeActionStartTimestamp(-1);

    handleMetrics();

    if (algorithmType == EMPIRIC)
        handleEmpiricAlgorithm(sonar);
    else if (algorithmType == DWA)
        handleDynamicWindowSonarData(sonar);
    else
        handleNeuralSonarData(sonar);
}

void ObstacleAvoidance::checkSonarData(const Data::SonarData &sonar)
{
    isSonarFrontObstacle = sonar.getFront(0) < SONAR_THRESHOLD || sonar.getFront(1) < SONAR_THRESHOLD || sonar.getFront(2) < SONAR_THRESHOLD ||sonar.getFront(3) < SONAR_THRESHOLD || sonar.getFront(4) < SONAR_THRESHOLD;
    isSonarBackObstacle = sonar.getBack(1) < SONAR_THRESHOLD || sonar.getBack(2) < SONAR_THRESHOLD || sonar.getBack(3) < SONAR_THRESHOLD;
    isSonarRightObstacle = sonar.getFront(3) < SONAR_THRESHOLD || sonar.getFront(4) < SONAR_THRESHOLD;
    isSonarLeftObstacle = sonar.getFront(0) < SONAR_THRESHOLD || sonar.getFront(1) < SONAR_THRESHOLD;
    // ldbg << "Sonar: Front is " << isSonarFrontObstacle << ", back is " << isSonarBackObstacle << ", left" << isSonarLeftObstacle << ", right" << isSonarRightObstacle << endl;

}

void ObstacleAvoidance::checkLaserData(double threshold)
{
    if (isLaser)
    {
        isLaserObstacle.clear();
        int angle = 0;
        for (int i=0; i<24; i++)
        {
            isLaserObstacle.append(checkLaserDataRange(angle, angle + 15, threshold));
            angle+=15;
        }

        isLaserBackObstacle =  isLaserObstacle[17] || isLaserObstacle[18];
        isLaserFrontObstacle = isLaserObstacle[3] || isLaserObstacle[5]|| isLaserObstacle[6] || isLaserObstacle[8];
        isLaserRightObstacle = isLaserObstacle[21] || isLaserObstacle[23] || isLaserObstacle[0]|| isLaserObstacle[2];
        isLaserLeftObstacle =  isLaserObstacle[9] || isLaserObstacle[11] || isLaserObstacle[12] || isLaserObstacle[14];

        westLaserDistance = laserReadings.at(170);
        northWestLaserDistance = laserReadings.at(120);
        northLaserDistance = laserReadings.at(90);
        northEastLaserDistance = laserReadings.at(40);
        eastLaserDistance = laserReadings.at(10);

        isLaser = false;
    }
}

int ObstacleAvoidance::getActualMovement(double leftSpeed, double rightSpeed)
{
    if (actualAction == NULL)
    {
        if (leftSpeed > rightSpeed)
            return RIGHT;
        else if (leftSpeed < rightSpeed)
            return LEFT;
        else
        {
            if (leftSpeed<0)
                return BACK;
            else if (leftSpeed>0)
                return FRONT;
            else
                return STOP;
        }
    }
    else
    {
        int type = actualAction->getType();
        double value = actualAction->getValue();
        if (type == Action::Rotation)
            value = fromRadiantToDegree(value);

        // ldbg <<"Obstacle Avoidance: Next action to do is a " << type << " with a value " << value<<endl;

        if (type == Action::Rotation && value>0)
            return LEFT;
        else if (type == Action::Rotation && value<0)
            return RIGHT;
        else if (type == Action::Translation && value<0)
            return BACK;
        else
            return FRONT;
    }
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

void ObstacleAvoidance::handleEmpiricAlgorithm(const Data::SonarData &sonar)
{
    checkLaserData(LASER_THRESHOLD);

    //ldbg << "Actual Movement is "<< actualMovementString[actualMovement] << ", empiric is "<< statusEnumString[empiricBehaviorStatus]<<", previous is "<<typeMovementEnumString[previousRotation] << endl;

    if(actualMovement == BACK)
    {
        handleEmpiricBack(actualAction, sonar);
    }
    else if (actualMovement == FRONT)
    {
        handleEmpiricFront(actualAction, sonar);
    }

    else if (actualMovement == RIGHT)
    {
        handleEmpiricRight(actualAction, sonar);
    }
    else if (actualMovement == LEFT)
    {
        handleEmpiricLeft(actualAction, sonar);
    }

}

void ObstacleAvoidance::handleEmpiricBack(const Data::Action *actualAction, const Data::SonarData &sonar)
{
    if(isSonarBackObstacle || isLaserBackObstacle)
    {
        if (empiricBehaviorStatus == DEACTIVATED)
        {
            emit sigHandleTimer();
            // ldbg <<"EMPIRIC (Back): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
            empiricBehaviorStatus = FIRSTTIME;

        }

        if (empiricBehaviorStatus > DEACTIVATED)
        {
            emit sigStopRobot(true);
            emit sigUpdateSonarData(sonar);
            empiricBehaviorStatus = EXEC_B;
            ldbg <<"EMPIRIC (Back): Go straight."<<endl;
            emit sigMoveRobot(0,EMP_GO_STRAIGHT,0);
        }

        emit sigChangeRobotControlType(NORMAL);
    }
    else if (empiricBehaviorStatus > DEACTIVATED && actualAction == NULL)
    {
        empiricBehaviorStatus = DEACTIVATED;
        // ldbg <<"EMPIRIC (Back): Nothing to do. Restart exploration."<<endl;
        emit sigRecomputePath(*actualFrontier);
    }
}

void ObstacleAvoidance::handleEmpiricFront(const Data::Action *actualAction, const Data::SonarData &sonar)
{
    if (isLaserFrontObstacle)
    {
        handleEmpiricFrontLaser();
    }
    else if (isSonarFrontObstacle)
    {
        handleEmpiricFrontSonar(sonar);
    }
    else
    {
        if (empiricBehaviorStatus> DEACTIVATED && actualAction == NULL)
        {
            empiricBehaviorStatus = DEACTIVATED;
            previousRotation = S;
            // ldbg <<"EMPIRIC (Front laser): Nothing to do. Restart exploration!" << endl;
            emit sigRecomputePath(*actualFrontier);
        }

    }
}

void ObstacleAvoidance::handleEmpiricFrontLaser()
{
    if (northLaserDistance<SONAR_THRESHOLD || northEastLaserDistance<SONAR_THRESHOLD|| northWestLaserDistance<SONAR_THRESHOLD)
    {
        emit sigStopRobot(true);
        ldbg <<"Case Sonar: Too close. Go back"<<endl;
        emit sigMoveRobot(0,EMP_GO_BACK,0);
    }
    else
    {
        if (northLaserDistance < LASER_THRESHOLD)
        {
            if (northWestLaserDistance<LASER_THRESHOLD)
            {
                if (northEastLaserDistance<LASER_THRESHOLD)
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case LFRS>: Rotate left max, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case LFR_S<: Rotate right max.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case LFR_L: Rotate left max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case LFR_R: Rotate right max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);

                    }
                }
                else
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case LFR_S>: Rotate left max, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case LFR_S<: Rotate right max.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case LFR_L: Rotate left max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case LFR_R: Rotate right max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                    }
                }
            }
            else
            {
                if (northEastLaserDistance<LASER_THRESHOLD)
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case FR_S>: Rotate left max, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case FR_S<: Go back, rotate right min.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(0,EMP_GO_BACK,0);
                            emit sigMoveRobot(EMP_ROTATERIGHT_MIN, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case FR_L: Rotate left max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case FR_R: Go back, Rotate right min, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = R;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(0,EMP_GO_BACK,0);
                        emit sigMoveRobot(EMP_ROTATERIGHT_MIN, EMP_GO_STRAIGHT,0);

                    }
                }
                else
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case F_S>: Rotate left max, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case F_S<: Rotate right max.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case F_L: Rotate left max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case F_R: Rotate right max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = R;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                    }
                }
            }
        }
        else
        {
            if (northWestLaserDistance<LASER_THRESHOLD)
            {
                if (northEastLaserDistance<LASER_THRESHOLD)
                {
                    emit sigStopRobot(true);
                    ldbg <<"Case LR: Go back ("<<frontierAngle<<")"<<endl;
                    previousRotation = S;
                    empiricBehaviorStatus = EXEC_F;
                    emit sigMoveRobot(0, EMP_GO_BACK,0);
                }
                else
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case R_S>: Rotate left max, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case R_S<: Rotate right max.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case R_L: Rotate left max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case R_R: Rotate left min, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATELEFT_MIN, EMP_GO_STRAIGHT,0);

                    }

                }
            }
            else
            {
                if (northEastLaserDistance<LASER_THRESHOLD)
                {
                    if (previousRotation == S)
                    {
                        if (frontierAngle>=0)
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case L_S>: Rotate left min, go straight ("<<frontierAngle<<")"<<endl;
                            previousRotation = L;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATELEFT_MIN, EMP_GO_STRAIGHT,0);
                        }
                        else
                        {
                            emit sigStopRobot(true);
                            ldbg <<"Case L_S<: Rotate right max.("<<frontierAngle<<")"<<endl;
                            previousRotation = R;
                            empiricBehaviorStatus = EXEC_F;
                            emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);
                        }

                    }
                    else if (previousRotation == L)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case L_L: Rotate left min, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATERIGHT_MIN, EMP_GO_STRAIGHT,0);
                    }
                    else if (previousRotation == R)
                    {
                        emit sigStopRobot(true);
                        ldbg <<"Case L_R: Rotate right max, ignore frontier angle("<<frontierAngle<<")"<<endl;
                        previousRotation = L;
                        empiricBehaviorStatus = EXEC_F;
                        emit sigMoveRobot(EMP_ROTATERIGHT_MAX, EMP_GO_STRAIGHT,0);

                    }
                }
                else
                {
                    ldbg <<"Case free: continue walking"<<endl;
                }

            }
        }
    }
    emit sigChangeRobotControlType(NORMAL);
}

void ObstacleAvoidance::handleEmpiricFrontSonar(const Data::SonarData &sonar)
{
    if (empiricBehaviorStatus == DEACTIVATED)
    {
        emit sigStopRobot(true);
        emit sigHandleTimer();
        //ldbg <<"EMPIRIC (Front sonar): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
        empiricBehaviorStatus = FIRSTTIME;
    }
    if (empiricBehaviorStatus  == FIRSTTIME)
    {
        emit sigUpdateSonarData(sonar);
        //ldbg <<"EMPIRIC (Front): Handle sonar" << endl;
        empiricBehaviorStatus = EXEC_F;
        emit sigMoveRobot(0,EMP_GO_BACK,0);
    }

    emit sigChangeRobotControlType(NORMAL);
}


void ObstacleAvoidance::handleEmpiricRight(const Data::Action *actualAction, const Data::SonarData &sonar)
{
    if (isLaserRightObstacle)
    {
        handleEmpiricRightLaser();
    }
    else if (isSonarRightObstacle)
    {
        handleEmpiricRightSonar(sonar);
    }
    else
    {
        if (empiricBehaviorStatus > DEACTIVATED && actualAction == NULL)
        {
            empiricBehaviorStatus = DEACTIVATED;
            previousRotation = S;
            //   ldbg <<"EMPIRIC (Right sonar): Nothing to do. Restart exploration!" << endl;
            emit sigRecomputePath(*actualFrontier);
        }

    }
}

void ObstacleAvoidance::handleEmpiricRightLaser()
{
    if (eastLaserDistance<LASER_THRESHOLD)
    {
        if (previousRotation!=L)
        {
            emit sigStopRobot(true);
            ldbg << "Case RR_SR: Rotate left max"<<endl;
            previousRotation = L;
            empiricBehaviorStatus = EXEC_R;
            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
        }
        else
        {
            ldbg <<"Case RR_L: Already rotating."<<endl;
        }
    }

    emit sigChangeRobotControlType(NORMAL);
}

void ObstacleAvoidance::handleEmpiricRightSonar(const Data::SonarData &sonar)
{
    if (empiricBehaviorStatus == DEACTIVATED)
    {
        emit sigStopRobot(true);
        emit sigHandleTimer();
        //ldbg <<"EMPIRIC (Right sonar): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
        empiricBehaviorStatus = FIRSTTIME;
    }
    if (empiricBehaviorStatus  == FIRSTTIME)
    {
        emit sigUpdateSonarData(sonar);
        // ldbg <<"EMPIRIC (Right sonar): Handle sonar" << endl;
        empiricBehaviorStatus = EXEC_R;
        emit sigMoveRobot(0,EMP_GO_STRAIGHT,0);
    }


    emit sigChangeRobotControlType(NORMAL);
}

void ObstacleAvoidance::handleEmpiricLeft(const Data::Action *actualAction, const Data::SonarData &sonar)
{
    if (isLaserLeftObstacle)
    {
        handleEmpiricLeftLaser();
    }
    else if (isSonarLeftObstacle)
    {
        handleEmpiricLeftSonar(sonar);
    }
    else
    {
        if (empiricBehaviorStatus > DEACTIVATED && actualAction == NULL)
        {
            empiricBehaviorStatus = DEACTIVATED;
            previousRotation = S;
            //ldbg <<"EMPIRIC (Left sonar): Nothing to do. Restart exploration!" << endl;
            emit sigRecomputePath(*actualFrontier);
        }

    }
}

void ObstacleAvoidance::handleEmpiricLeftLaser()
{
    if (westLaserDistance<LASER_THRESHOLD)
    {
        if (previousRotation!=R)
        {
            emit sigStopRobot(true);
            ldbg << "Case LL_SL: Rotate right max"<<endl;
            previousRotation = R;
            empiricBehaviorStatus = EXEC_R;
            emit sigMoveRobot(EMP_ROTATELEFT_MAX, EMP_GO_STRAIGHT,0);
        }
        else
        {
            ldbg <<"Case LL_R: Already rotating."<<endl;
        }
    }

    emit sigChangeRobotControlType(NORMAL);
}

void ObstacleAvoidance::handleEmpiricLeftSonar(const Data::SonarData &sonar)
{
    if (empiricBehaviorStatus == DEACTIVATED)
    {
        emit sigStopRobot(true);
        emit sigHandleTimer();
        //ldbg <<"EMPIRIC (Left sonar): Stop robot. DEACTIVATED to FIRSTTIME."<<endl;
        empiricBehaviorStatus = FIRSTTIME;
    }
    if (empiricBehaviorStatus  == FIRSTTIME)
    {
        emit sigUpdateSonarData(sonar);
        // ldbg <<"EMPIRIC (Left laser): Handle sonar" << endl;
        empiricBehaviorStatus = EXEC_L;
        emit sigMoveRobot(0,EMP_GO_STRAIGHT,0);
    }
    emit sigChangeRobotControlType(NORMAL);
}

//DWA
void ObstacleAvoidance::handleDynamicWindowSonarData(const Data::SonarData &sonar)
{
    emit sigChangeActionStartTimestamp(-1);

    isLaserObstacleDWA = false;

    if (isLaser)
    {
        isLaser = false;
        isLaserObstacleDWA = checkLaserDataRange(0,360,LASER_THRESHOLD);
    }

    if (isLaserObstacleDWA)
    {
        //ldbg << QTime::currentTime().toString()<<": State is "<< statusEnumString[dwaBehaviorStatus]<< endl;

        if (dwaBehaviorStatus == DEACTIVATED)
        {
            emit sigStopRobot(true);
            emit sigHandleTimer();
            dwaBehaviorStatus = FIRSTTIME;
            // ldbg << QTime::currentTime().toString()<<": DEACTIVATED to FIRSTIME (Inner)"<< endl;
        }

        if (dwaBehaviorStatus > DEACTIVATED)
        {
            emit sigUpdateSonarData(sonar);

            QTime t;
            t.start();

            calculateSearchSpace();

            int bestValue = calculateBestVelocity();

            bestLeftSpeed = searchSpaceVelocities[bestValue].first;
            bestRightSpeed = searchSpaceVelocities[bestValue].second;

            bestX = searchSpacePoses[bestValue].x + actualPose.getX();
            bestY = searchSpacePoses[bestValue].y + actualPose.getY();
            bestTheta = searchSpacePoses[bestValue].localHeading + actualPose.getTheta();

            bestPose = Pose(bestX, bestY, bestTheta);

            ldbg << "DWA: Best pose is "<<bestPose << ". Best velocity is (" << bestLeftSpeed << ", " << bestRightSpeed << "). Elapsed time "<<t.elapsed() << endl;

            dwaBehaviorStatus = EXEC;

            emit sigDoMovement(bestLeftSpeed , bestRightSpeed);
        }
        emit sigChangeRobotControlType(NORMAL);
    }
    else
    {
        bestDistance =  actualPose.getDistance(bestPose);
        int bestRotation =  fromRadiantToDegree(computeRotationFromPoses(actualPose,bestPose));
        //ldbg <<"DWA: (" << bestDistance <<"," <<bestRotation<<")" << endl;
        if (dwaBehaviorStatus > DEACTIVATED && bestDistance<0.3 && (bestRotation>-EMP_ANGLE_TOL || bestRotation<EMP_ANGLE_TOL))
        {

            dwaBehaviorStatus = DEACTIVATED;
            //ldbg <<"DWA: Restart exploration."<< endl;
            emit sigRecomputePath(*actualFrontier);

        }
    }
}

bool ObstacleAvoidance::checkLaserDataRange(int angleMin, int angleMax, double threshold)
{
    bool isLaserObstacle = false;
    for (int i = angleMin; i < angleMax;i++)
    {
        double actualDistance = laserReadings.at(i);
        //ldbg << "DWA - Laser capture: ("<< i << " " << actualDistance << "). Min is "<< threshold << endl;
        if (actualDistance < threshold)
            isLaserObstacle = true;

    }

    if (isLaserObstacle)
    {
        for (int i = angleMin; i < angleMax;i++)
        {
            double actualDistance = laserReadings.at(i);
            //ldbg << "DWA - Laser capture: ("<< i << " " << actualDistance << ")"<< endl;

        }
    }


    return isLaserObstacle;
}

bool ObstacleAvoidance::checkSafePose(int laserID, LocalMapEl actualSample)
{
    bool isSafePose = true;
    for (int safeLaser = laserID - DWA_ROTATION_SAFETY; safeLaser<=laserID + DWA_ROTATION_SAFETY; safeLaser++)
    {
        int fixAngle = safeLaser;
        if (safeLaser<0)
            fixAngle = safeLaser + 360;
        if (safeLaser>=360)
            fixAngle = safeLaser - 360;

        angleLaserDistance = laserReadings.at(fixAngle);
        actualDistance = actualSample.globalDistance;

        //ldbg<<"DWA - Safe pose: ("<< fixAngle << " " << actualDistance <<" "<< laserDistance<< ")"<<endl;
        if (actualDistance + DWA_TRANSLATION_SAFETY> angleLaserDistance)
            isSafePose = false;
    }

    return isSafePose;
}

void ObstacleAvoidance::calculateSearchSpace()
{
    searchSpacePoses.clear();
    searchSpaceVelocities.clear();

    QTime t;
    t.start();

    //ldbg << "========= CALCULATE SEARCH SPACE =========" << endl;

    ldbg << "DWA: Actual pose is "<< actualPose << endl;

    for (double i=DWA_MIN_VELOCITY; i<=DWA_MAX_VELOCITY; i+=DWA_STEP)
    {
        for (double j=DWA_MIN_VELOCITY; j<=DWA_MAX_VELOCITY; j+=DWA_STEP)
        {
            LocalMapEl actualSample;

            Pose predictedPose = forwardKinematics(Pose(0,0,0),i,j,DWA_TIME);
            actualSample.x = predictedPose.getX();
            actualSample.y = predictedPose.getY();
            actualSample.localHeading = predictedPose.getTheta();
            actualSample.localAngle = atan2(predictedPose.getY(),predictedPose.getX());
            actualSample.globalHeading = actualSample.localHeading + actualPose.getTheta();
            actualSample.globalAngle = atan2(predictedPose.getY() + actualPose.getY(), predictedPose.getX() + actualPose.getX());
            actualSample.globalDistance = predictedPose.getDistance(Pose(0,0,0));

            int laserID = fromRadiantToDegree(actualSample.localAngle);
            if (laserID<0)
                laserID+=360;


            //ldbg << "DWA - Search space: "<< laserID<<" ("<<i <<", "<< j<<
            //      ") "<< actualPose << ", " << "{" << actualSample.x + actualPose.getX()<< ","<<actualSample.y + actualPose.getY()<<
            //    ","<<actualSample.globalHeading <<","<<actualSample.globalAngle <<","<<actualSample.localHeading<<","<<actualSample.localAngle <<","<<actualSample.globalDistance<<"}";

            safePose = checkSafePose(laserID, actualSample);

            //ldbg <<" -> " << safePose<<endl;

            if (safePose)
            {
                //ldbg << QTime::currentTime().toString()<<": Safe pose. Append velocities "<< i << ", " << j <<endl;
                QPair<double,double> velocity(i,j);
                searchSpaceVelocities.append(velocity);
                searchSpacePoses.append(actualSample);
            }

        }
    }
    //ldbg <<QTime::currentTime().toString()<<": Search space end in " <<t.elapsed()<<" milliseconds." << endl;
}

int ObstacleAvoidance::calculateBestVelocity()
{
    double velocity= 0;
    double targetHeading = 0;
    double clearance = 0;
    int bestValue = 0;
    double bestCost = 10;

    //ldbg << "========= CALCULATE BEST VELOCITY =========" << endl;

    QTime t;
    t.start();

    //ldbg << "DWA - Actual frontier (pre translate):"<<actualPose <<" , "<< actualFrontier <<endl;
    double fx = actualFrontier->getX()-actualPose.getX();
    double fy = actualFrontier->getY()-actualPose.getY();
    double rotationAngle = -actualPose.getTheta();

    double fxRot = fx*cos(rotationAngle)-fy*sin(rotationAngle);
    double fyRot = fx*sin(rotationAngle)+fy*cos(rotationAngle);
    double fThetaRot = atan2(fyRot,fxRot);

    Pose actualFrontierDWA(fxRot,fyRot,fThetaRot);


    //ldbg << QTime::currentTime().toString()<<": Search space size = " << searchSpaceVelocities.size() << ", Search poses size = " << searchSpacePoses.size() <<
    //     ", Obstacle size = " << laserReadings.size() << endl;

    //ldbg <<"DWA - Actual pose local rotation is "<< wrapDeg(fromRadiantToDegree(actualPose.getTheta()))<<", global rotation is "<<wrapDeg(fromRadiantToDegree(actualPoseGlobalRot)) << endl;
    ldbg <<"DWA - Actual frontier: " << actualFrontierDWA<<endl;

    if (searchSpaceVelocities.size()<0)
        return 0;

    for(int counter = 0; counter < searchSpaceVelocities.size(); counter++)
    {
        Pose predictedPose (searchSpacePoses.at(counter).x, searchSpacePoses.at(counter).y, searchSpacePoses.at(counter).localHeading);

        int laserID = fromRadiantToDegree(searchSpacePoses.at(counter).localAngle);

        if (laserID<0)
            laserID+=360;

        double obstacleDistance = laserReadings.at(laserID);
        double predictedPoseDistance = searchSpacePoses.at(counter).globalDistance;

        clearance = obstacleDistance - predictedPoseDistance;
        targetHeading = computeRotationFromPoses(predictedPose,actualFrontierDWA);
        velocity = (searchSpaceVelocities.at(counter).first + searchSpaceVelocities.at(counter).second)/2;

        //ldbg <<"DWA - parameters (real): "<< predictedPose << " ("<< counter << " " << targetHeading << " " << clearance << " " << velocity <<")";

        double t = abs(targetHeading)/2*M_PI;
        double c = predictedPoseDistance/obstacleDistance;
        double v = 1-velocity;

        //ldbg <<"DWA - parameters (normalized): ("<< counter << " " << t << " " << c<< " " << v <<")";

        double cost = DWA_SIGMA*(DWA_ALPHA*t + DWA_BETA*c + DWA_GAMMA*v);
        //ldbg << " -> " << cost << endl;
        if (cost<=bestCost)
        {
            bestCost = cost;
            bestValue = counter;
        }
    }


    //ldbg <<QTime::currentTime().toString()<<": Calculate velocity end in " <<t.elapsed()<<" milliseconds." << endl;

    //ldbg <<"DWA - best parameters: ("<< bestValue << " " << fromRadiantToDegree(targetHeading) << " " << clearance << " " << velocity<<endl;
    //ldbg <<"DWA - best cost:" <<bestCost <<endl;

    return bestValue;
}

//Neural
void ObstacleAvoidance::handleNeuralSonarData(const Data::SonarData &sonar)
{

    emit sigChangeActionStartTimestamp(-1);

    if (isSonarFrontObstacle || isSonarBackObstacle)
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
            emit sigRecomputePath(*actualFrontier);
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
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MAX);
        break;
    }
    case LLLFR:
    {
        ldbg <<"FANN: Go back and rotate to right (max)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MAX);
        break;
    }
    case LLLF:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MIN);
        break;
    }
    case LLL:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MIN);
        break;
    }
    case LL:
    {
        ldbg <<"FANN: Go back and rotate to right (min)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MIN);
        break;
    }
    case LFRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MAX);
        break;
    }
    case LFR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MAX);
        break;
    }
    case LF:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    case L:
    {
        ldbg <<"FANN: Go back and rotate to right (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATERIGHT_MIN);
        break;
    }
    case FRRR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    case FR:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    case F:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    case RRR:
    {
        ldbg <<"FANN: Go back and rotate to left (max)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MAX);
        break;
    }
    case RR:
    {
        ldbg <<"FANN: Go back and rotate to left (min)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    case R:
    {
        ldbg <<"FANN: Go back and rotate to left (med)" << endl;
        emit sigMoveRobot(0,EMP_GO_BACK,EMP_ROTATELEFT_MIN);
        break;
    }
    default:
    {
        ldbg <<"FANN: Prediction error."<<endl;
        break;
    }
    }
    emit sigMoveRobot(0,EMP_GO_STRAIGHT,0);
}

void ObstacleAvoidance::setLaser(Data::LaserData &laser)
{
    isLaser = true;
    //ldbg << "New laser coming at "<< QTime::currentTime().toString();
    actualLaser = new LaserData(laser.getTimestamp(),laser.getFOV(),laser.getResolution(), laser.getReadings());
    laserReadings.clear();

    for (int i=0;i<360;i++)
    {
        int laserID = i + 90;
        if (laserID < 0)
            laserID = laserID + 360;
        if (laserID>360)
            laserID = laserID - 360;
        laserReadings.append(laser.getReadings()[laserID]);

        // ldbg << i << ", " << laserReadings.at(i) << ", " << laser.getReadings()[i]<<endl;
    }

    //ldbg << ". Actual laser timestamp " << actualLaser->getTimestamp()<<endl;
}






