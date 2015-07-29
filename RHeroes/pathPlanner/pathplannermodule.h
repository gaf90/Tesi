#ifndef PATHPLANNERMODULE_H
#define PATHPLANNERMODULE_H

#include <QObject>
#include "slam/map.h"
#include <QStack>
#include "data/action.h"
#include "data/robotstate.h"
#include "slam/geometry/point.h"
#include "shared/mutexqueue.h"
#include "slam/slammodule.h"
#include "data/wirelessmessage.h"
#include "pathplannerhelper.h"
#include <QMutex>


namespace PathPlanner {

    enum PathPlannerTasks{
        FRONTIER,
        VICTIMS
    };

    /**
    * This class represents the module for PathPlanning.
    */
    class PathPlannerModule : public QThread
    {
        Q_OBJECT
    public:
        explicit PathPlannerModule(uint identifier, bool isKenaf,QObject *parent = 0);
        virtual ~PathPlannerModule();

        /**
          * This method compute a path for the robot to reach the pose passed.
          * It emits the signal sigPerformActionPM to indicate the sequence of actions that
          * have to be performed.
          * NOTE: the method could fail and find no path at all. In this case, no signal is emitted.
          * @param pose the pose to reach
          * @param map in which the robot is.
          */
        void computePath(const Data::Pose &pose,const SLAM::Map &map);
        void setSlamModule(SLAM::SLAMModule *slamModule);
        void setRobotState(Data::RobotState *robotState);
        /**
          * This method compute a path for the robot to reach the pose passed.
          * Then it compute an estimate of how many time it takes to perform the plan.
          * NOTE: the method could fail and find no path at all.
          * @param task if the path planner should estimate a duration to reach a victim or a frontier.
          * @param pose the pose to reach
          * @return the time needed to perform the task. -1 in case of failure.
          */
        double estimateTimeForAPlan(PathPlannerTasks task, const Data::Pose &pose);
    signals:
        void sigStopRobotForPlanningPM();
        void sigPerformActionPM(PathPlanner::AbstractAction *action);
        void sigRestartExplorationPM();
        void sigPathNotFoundPM(const Data::Message &);
        void sigHandleBandleFrontierPM(const Data::Pose);
        void sigFrontierToReachPM(Data::Pose);
        void sigCleanBadFrontiersPM();
        void sigPathNotFound();
    public slots:
        void onPointToReachPP(double x, double y);
        void changeStatus(bool enable);
        void run();
        void sendMessageToGui();
    private:
        void stopComputation();
        void postProcessPlan(QStack<AbstractAction *> *plan);
        QList<Data::Pose> postProcessHybridAStar(QStack<AbstractAction *> *plan, const SLAM::Map &map);

        uint identifier;
        Shared::MutexQueue<Data::Pose *> *poseQueue;
        QMutex *pathPlannerMutex;
        bool endComputation;
        SLAM::SLAMModule *slamModule;       
        SLAM::Geometry::Point lastPoint;
        SLAM::Map slamMap;
        Data::RobotState *robotState;
        bool isKenaf;

        bool enabled;
        QMutex enablingMutex;
        PathPlannerHelper pathPlannerHelper;

    };
}



#endif // PATHPLANNERMODULE_H
