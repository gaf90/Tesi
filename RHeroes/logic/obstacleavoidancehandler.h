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


#define THRESHOLD 0.23
#define RUOTASINISTRA_MIN 10
#define RUOTASINISTRA_MED 20
#define RUOTASINISTRA_MAX 40
#define RUOTADESTRA_MIN -10
#define RUOTADESTRA_MED -20
#define RUOTADESTRA_MAX -40
#define VAI_AVANTI 0.5
#define VAI_INDIETRO -0.2

#define RADIUS_LOCAL 1
#define SEARCH_SPACE_GRANULARITY 360
#define DYNAMIC_DELTA_T 0.1



class ObstacleAvoidance : public QObject
{
    Q_OBJECT
public:

    int reactiveFrontBehaviorStatus, reactiveBackBehaviorStatus;

    explicit ObstacleAvoidance(InverseKinematic* i, QObject *parent = 0);
    void setSlamModule(SLAM::SLAMModule *slam);

    void handleNeuralSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier);
    void handleEmpiricSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction, Data::Pose *actualFrontier);
    void handleDynamicWindowSonarData(const Data::SonarData &sonar,Data::RobotState *actualState,const Data::Action*actualAction,  Data::Pose *actualFrontier);

public slots:
    void setMovementType(int type);


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
    QVector<QPair<QPair<double, double>, int> > getLocalMap(const Data::Pose actualPose);
    QVector<QPair<double, double> > getLocalReachableSearchSpace(QVector<QPair<QPair<double, double>, int> > localMap);
    int calculateBestVelocity(QVector<QPair<double, double> > searchSpace);
    void handleFrontSonarData(const Data::SonarData &sonar);
    int getActualMovement(double leftSpeed, double rightSpeed);
    void handleFrontObstacle(const Data::SonarData &sonar);
    void obstacleAvoidanceEmpiricHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL);
    void handleBackObstacle(const Data::SonarData &sonar);
    void handleBackSonarData(const Data::SonarData &sonar);
    Data::Pose forwardKinematics(const Data::Pose &from, double vr, double vl, double time);

private:
    enum reactiveBehaviorEnum {DEACTIVATED,FIRSTTIME,EXEC};
    enum typeMovementEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
    enum movementStateEnum {FRONT,RIGHT,LEFT,BACK,STOP};
    enum controlTypeEnum {HYBRID,NORMAL};
    movementStateEnum actualMovement;
    typeMovementEnum typeMovement;

    Data::RobotState *actualState;
    const Data::Action*actualAction;
    Data::Pose *actualFrontier;

    InverseKinematic *inverseKinematicModule;

    SLAM::SLAMModule *slam;


};

#endif // OBSTACLEAVOIDANCEHANDLER_H
