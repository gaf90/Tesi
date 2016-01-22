#ifndef OBSTACLEAVOIDANCEHANDLER_H
#define OBSTACLEAVOIDANCEHANDLER_H

#include "inversekinematic.h"

#include <QObject>

#include <data/action.h>
#include <data/pose.h>
#include <data/robotstate.h>
#include <data/sonardata.h>

#include <pathPlanner/hybridAStar/hybridastaralgorithm.h>

#include <slam/slammodule.h>
#include <libraries/fann/doublefann.h>

#define THRESHOLD 0.22
#define LASER_THRESHOLD 0.5
#define LASER_MAX_RANGE 10
#define DWA_TIME 1
#define DWA_MAX_VELOCITY 0.5
#define DWA_STEP 0.2
#define DWA_SAFETY 5
#define DWA_DISTANCE 0.05

#define DWA_SIGMA 1
#define DWA_TARGET 1
#define DWA_CLEARANCE 0.5
#define DWA_VELOCITY 0.5


#define ANGLE_TOL 15
#define ROTATELEFT_MIN 5 + ANGLE_TOL
#define ROTATELEFT_MED 10 + ANGLE_TOL
#define ROTATELEFT_MAX 20 + ANGLE_TOL
#define ROTATERIGHT_MIN -5 - ANGLE_TOL
#define ROTATERIGHT_MED -10 - ANGLE_TOL
#define ROTATERIGHT_MAX -20 - ANGLE_TOL
#define GO_STRAIGHT 1
#define GO_BACK -0.3

#define RADIUS_LOCAL 3
#define SEARCH_SPACE_GRANULARITY 36
#define RH_RADIUS 0.63


class ObstacleAvoidance : public QObject
{
    Q_OBJECT
public:
    struct LocalMapEl{
        double angle;
        double x;
        double y;
        double distance;
    };


    int empiricFrontStatus, empiricBackStatus, dwaBehaviorStatus, neuralBehaviorStatus;

    explicit ObstacleAvoidance(InverseKinematic* i, int angleTol, QObject *parent = 0);
    void setSlamModule(SLAM::SLAMModule *slam);
    void handleObstacle(const Data::SonarData &sonar, Data::RobotState *actualState, const Data::Action *actualAction, Data::Pose *actualFrontier);
    void handleNeuralNetwork();
    void applyPredictedAction(int predictedMovement);
    void setLaser(Data::LaserData &laser);
    void checkSonarData(const Data::SonarData &sonar);
    bool checkLaserData();
    int getLaserID(double angle);
    bool checkSafePose(int laserID, LocalMapEl actualSample);
signals:
    void sigChangeActionStartTimestamp(int);
    void sigChangeRobotControlType(int);
    void sigUpdateSonarData(const Data::SonarData);
    void sigMoveRobot(double,double,double);
    void sigStopRobot(bool);
    void sigRestartExploration();
    void sigDoMovement(double, double);

private slots:
    void calculateSearchSpace(Data::Pose actualPose);
    int calculateBestVelocity();
    int getActualMovement(double leftSpeed, double rightSpeed);
    void empiricObstacleHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL);
    Data::Pose forwardKinematics(const Data::Pose &from, double vr, double vl, double time);

    void handleNeuralSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier);
    void handleEmpiricSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier);
    void handleDynamicWindowSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction,  Data::Pose *actualFrontier);
    void setMovementType(int type);

private:
    enum obstacleAlgEnum {EMPIRIC,DWA,NEURAL};
    enum algStateEnum {DEACTIVATED,FIRSTTIME,EXEC};
    enum movementStateEnum {FRONT,RIGHT,LEFT,BACK,STOP};
    enum controlTypeEnum {HYBRID,NORMAL};
    enum sensorDataEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
    struct fann* neuralNetwork;
    int oldPredictedMovement;
    int predictedMovement;


    movementStateEnum actualMovement;
    sensorDataEnum sensorDataCaptured;
    obstacleAlgEnum algorithmType;

    Data::RobotState *actualState;
    Data::LaserData *actualLaser;
    const Data::Action*actualAction;
    Data::Pose *actualFrontier;
    Data::Pose actualPose, bestPose;

    InverseKinematic *inverseKinematicModule;

    SLAM::SLAMModule *slam;


    QList<double> laserReadings;
    QList<LocalMapEl> searchSpacePoses;
    QList<QPair<double,double> > searchSpaceVelocities;

    int num_input;
    int num_output;
    int num_layers;
    int num_neurons_hidden;
    float desired_error;
    int max_epochs;
    int epochs_between_reports;

    int angleTol;

    bool isSonarObstacle;
    bool isBackObstacle;
    bool isLaser;

    double laserDistance;
    double actualDistance;
    bool safePose;
    double bestLeftSpeed;
    double bestRightSpeed;

    double bestX;
    double bestY;
    double bestTheta;

};

#endif // OBSTACLEAVOIDANCEHANDLER_H
