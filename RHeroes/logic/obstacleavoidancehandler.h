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
#include <slam/geometry/frontier.h>

#define SONAR_THRESHOLD Config::OBS::sonar_threshold
#define LASER_THRESHOLD Config::OBS::laser_threshold

#define EMP_ANGLE_TOL Config::OBS::emp_angle_tolerance
#define EMP_GO_STRAIGHT Config::OBS::emp_straight_meters
#define EMP_GO_BACK Config::OBS::emp_back_meters
#define EMP_ROTATELEFT_MIN Config::OBS::emp_min_rotation + EMP_ANGLE_TOL
#define EMP_ROTATELEFT_MAX Config::OBS::emp_max_rotation + EMP_ANGLE_TOL
#define EMP_ROTATERIGHT_MIN -Config::OBS::emp_min_rotation - EMP_ANGLE_TOL
#define EMP_ROTATERIGHT_MAX -Config::OBS::emp_max_rotation - EMP_ANGLE_TOL

#define DWA_MIN_VELOCITY Config::OBS::dwa_min_velocity
#define DWA_MAX_VELOCITY Config::OBS::dwa_max_velocity
#define DWA_STEP Config::OBS::dwa_step
#define DWA_TIME Config::OBS::dwa_time
#define DWA_ROTATION_THRESHOLD Config::OBS::dwa_rotation_threshold
#define DWA_TRANSLATION_THRESHOLD Config::OBS::dwa_translation_threshold
#define DWA_ROTATION_SAFETY Config::OBS::dwa_rotation_safety
#define DWA_TRANSLATION_SAFETY Config::OBS::dwa_translation_safety
#define DWA_SIGMA Config::OBS::dwa_sigma
#define DWA_ALPHA Config::OBS::dwa_alpha_target
#define DWA_BETA Config::OBS::dwa_beta_clearance
#define DWA_GAMMA Config::OBS::dwa_gamma_velocity



class ObstacleAvoidance : public QObject
{
    Q_OBJECT
public:
    struct LocalMapEl{
        double localHeading;
        double globalHeading;
        double localAngle;
        double globalAngle;
        double x;
        double y;
        double globalDistance;
        double rightSpeed;
        double leftSpeed;
    };


    int empiricBehaviorStatus, dwaBehaviorStatus, neuralBehaviorStatus;

    explicit ObstacleAvoidance(InverseKinematic* i, int angleTol, QObject *parent = 0);
    void setSlamModule(SLAM::SLAMModule *slam);
    void handleObstacle(const Data::SonarData &sonar, Data::RobotState*pastState, Data::RobotState *actualState, const Data::Action *actualAction, Data::Pose *actualFrontier);
    void handleNeuralNetwork();
    void applyPredictedAction(int predictedMovement);
    void setLaser(Data::LaserData &laser);
    void checkSonarData(const Data::SonarData &sonar);
    bool checkLaserDataRange(int angleMin, int angleMax, double threshold);
    bool checkSafePose(int laserID, LocalMapEl actualSample);
    void checkLaserData(double threshold);
    void handleEmpiricBack(const Data::Action *actualAction, const Data::SonarData &sonar);
    void handleEmpiricFront(const Data::Action *actualAction, const Data::SonarData &sonar);
    void handleEmpiricRight(const Data::Action *actualAction, const Data::SonarData &sonar);
    void handleEmpiricLeft(const Data::Action *actualAction, const Data::SonarData &sonar);
    void handleEmpiricFrontLaser();
    void handleEmpiricFrontSonar(const Data::SonarData &sonar);
    void handleEmpiricRightLaser();
    void handleEmpiricRightSonar(const Data::SonarData &sonar);
    void handleEmpiricLeftLaser();
    void handleEmpiricLeftSonar(const Data::SonarData &sonar);
    void handleMetrics();
    Data::Pose calcLocalFrontier(Data::Pose actualFrontier, Data::Pose centerPose);
signals:

    void sigChangeActionStartTimestamp(int);
    void sigChangeRobotControlType(int);
    void sigUpdateSonarData(const Data::SonarData);
    void sigMoveRobot(double,double,double);
    void sigStopRobot(bool);
    void sigRecomputePath(Data::Pose);
    void sigDoMovement(double, double);
    void sigHandleTimer();
    void sigFrontierReached();

private slots:
    void calculateSearchSpace();
    int calculateBestVelocity();
    int getActualMovement(double leftSpeed, double rightSpeed);
    Data::Pose forwardKinematics(const Data::Pose &from, double vr, double vl, double time);

    void handleNeuralSonarData(const Data::SonarData &sonar);
    void handleEmpiricAlgorithm(const Data::SonarData &sonar);
    void handleDynamicWindowSonarData(const Data::SonarData &sonar);
    void setMovementType(int type);


private:
    enum obstacleAlgEnum {EMPIRIC,DWA,NEURAL};
    enum algStateEnum {DEACTIVATED,FIRSTTIME,EXEC,EXEC_F,EXEC_B,EXEC_R,EXEC_L};
    enum movementStateEnum {FRONT,RIGHT,LEFT,BACK,STOP};
    enum controlTypeEnum {HYBRID,NORMAL};
    enum sensorDataEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
    struct fann* neuralNetwork;
    int oldPredictedMovement;
    int predictedMovement;


    movementStateEnum actualMovement;
    sensorDataEnum previousRotation;
    obstacleAlgEnum algorithmType;

    Data::RobotState *actualState,*pastState;
    Data::LaserData* actualLaser;
    const Data::Action*actualAction;
    Data::Pose *actualFrontier;
    Data::Pose localFrontier, previousActualPose, actualPose, bestPose;
    double frontierAngle;

    InverseKinematic *inverseKinematicModule;

    SLAM::SLAMModule *slam;


    QList<LocalMapEl> searchSpace;

    int num_input;
    int num_output;
    int num_layers;
    int num_neurons_hidden;
    float desired_error;
    int max_epochs;
    int epochs_between_reports;

    int angleTol;

    bool isSonarFrontObstacle, isSonarBackObstacle, isSonarRightObstacle, isSonarLeftObstacle;
    QList<bool> isLaserObstacle;
    QList<double> laserReadings;
    bool isLaserFrontObstacle, isLaserBackObstacle, isLaserRightObstacle, isLaserLeftObstacle, isLaserObstacleDWA;
    bool haveNewLaserData;

    double northLaserDistance, northWestLaserDistance, northEastLaserDistance, eastLaserDistance, westLaserDistance;
    bool safePose;

    double bestLeftSpeed;
    double bestRightSpeed;
    double bestDistance;
    double bestX;
    double bestY;
    double bestTheta;

    double meters;

};

#endif // OBSTACLEAVOIDANCEHANDLER_H
