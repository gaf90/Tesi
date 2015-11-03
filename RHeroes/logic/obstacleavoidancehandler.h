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
#define LASER_THRESHOLD 0.4
#define RUOTASINISTRA_MIN 20
#define RUOTASINISTRA_MED 30
#define RUOTASINISTRA_MAX 40
#define RUOTADESTRA_MIN -20
#define RUOTADESTRA_MED -30
#define RUOTADESTRA_MAX -40
#define VAI_AVANTI 0.5
#define VAI_INDIETRO -0.15

#define RADIUS_LOCAL 3
#define SEARCH_SPACE_GRANULARITY 20
#define DYNAMIC_DELTA_T 1.5
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

    explicit ObstacleAvoidance(InverseKinematic* i, QObject *parent = 0);
    void setSlamModule(SLAM::SLAMModule *slam);
    void handleObstacle(const Data::SonarData &sonar, Data::RobotState *actualState, const Data::Action *actualAction, Data::Pose *actualFrontier);
    void handleNeuralNetwork();
    void applyPredictedAction(int predictedMovement);
    void setLaser(Data::LaserData &laser);


    void checkSonarData(const Data::SonarData &sonar);
signals:
    void sigChangeActionStartTimestamp(int);
    void sigChangeRobotControlType(int);
    void sigUpdateSonarData(const Data::SonarData);
    void sigMoveRobot(double,double,double);
    void sigStopRobot(bool);
    void sigRestartExploration();

private slots:
    bool isReachablePose(Data::Pose predictedPose, Data::Pose actualPose);
    QVector<QPair<double, double> > calculateSearchSpace(const Data::SonarData &sonar);
    QVector<ObstacleAvoidance::LocalMapEl> getLocalMap(const Data::Pose actualPose, QList<double> actualLaser);
    QVector<QPair<double, double> > getLocalReachableSearchSpace();
    int calculateBestVelocity(QVector<QPair<double, double> > searchSpace);
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

    InverseKinematic *inverseKinematicModule;

    SLAM::SLAMModule *slam;

    QVector<LocalMapEl> localMap;

    int num_input;
    int num_output;
    int num_layers;
    int num_neurons_hidden;
    float desired_error;
    int max_epochs;
    int epochs_between_reports;

    bool isFrontObstacle;
    bool isBackObstacle;
    bool isLaser;


};

#endif // OBSTACLEAVOIDANCEHANDLER_H
