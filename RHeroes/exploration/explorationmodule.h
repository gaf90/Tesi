#ifndef EXPLORATIONMODULE_H
#define EXPLORATIONMODULE_H

#include <QThread>
#include <QTimer>
#include <QMutex>
#include "slam/slammodule.h"
#include "slam/map.h"
#include "exploration/evaluationfunction.h"
#include "coordination/coordinationmodule.h"
#include "semanticMapping/semanticmappingmodule.h"
#include "data/robotstate.h"
#include "PRM/evaluationPRM/prmfunction.h"
#include "PRM/pathplannerprm.h"

namespace Exploration {
    class ExplorationModule : public QThread
    {
        Q_OBJECT
    public:
        /**
          * Constructor for the exploration module. This constructor takes almost all the needed parameters.
          * Each of the objects must be created befare the call of this constructor.
          * @param robotId the robot identifier.
          * @param type the evaluation function the module should use.
          * @param slamModule the slamModule.
          * @param state the state of the robot.
          * @param semMapModule the semantic mapping module.
          * @param coordModule the coordination module.
          */
        explicit ExplorationModule(
                uint robotId, EvalType type, SLAM::SLAMModule *slamModule, Data::RobotState *state,
                SemanticMapping::SemanticMappingModule *semMapModule,
                Coordination::CoordinationModule *coordModule, QObject *parent = 0);

        /**
          * Constructor for the exploration module. This constructor does not initialize the
          * parameters needed for the execution. So you muste set them with the appropriate setters.
          * @param robotId the robot identifier.
          * @param type the evaluation function the module should use.
          */
        explicit ExplorationModule(uint robotId, EvalType type, QObject *parent = 0);

        /**
          * Destroyer for the module.
          * the destroyer does not delete the other modules and the robot state.
          */
        virtual ~ExplorationModule();

        /**
          * Setter for the SLAM module. The object does not take the ownership of the parameter.
          * @param slamModule.
          */
        void setSLAMModule(SLAM::SLAMModule *slamModule);

        /**
          * Setter for the Semantic Mapping module. The object does not take the ownership of the parameter.
          * @param semModule.
          */
        void setSemModule(SemanticMapping::SemanticMappingModule *semModule);

        /**
          * Setter fot the Coordination module. The object does not take the ownership of the parameter.
          * @param coordModule.
          */
        void setCoordModule(Coordination::CoordinationModule *coordModule);
        /**
          * Setter for the Robot State. The object does not take the ownership of the parameter.
          * @param state.
          */
        void setRobotState(Data::RobotState *state);
    signals:
        /**
          * Signal used to send a Power Request to the WSS.
          * @param msg the Wireless Messege with the request.
          */
        void sigDriverMessageEM(const Data::Message &msg);

        void sigStartNewAuctionEM(EvaluationRecords *evalRec, bool forceAssignment);

        void powerSignalDataGathered();

        void sigStartTimerEM();

        void sigStopTimerEM();

        void sigNoFrontierAvailableEM();

        //PRM
        void sigPerformActionEM(PathPlanner::AbstractAction* action);

        void sigStopRobotForPlanningEM();

        void sigRestartExplorationEM();

        void sigFrontierToReachEM(Data::Pose);
        //

    public slots:     
        /**
          * Method used by other modules to ask for the evaluation of a frontier.
          * @param frontier to evaluate.
          * @return the utility of the frontier.
          */
        double evaluateFrontier(const SLAM::Geometry::Frontier *frontier);


        /**
          * Method used by other modules to ask for the evaluation of a list of frontiers.
          * @param frontiers the list to evaluate.
          * @return an evaluation records that contains, for each frontier, its evaluation.
          */
        EvaluationRecords* evaluateFrontiers(const QList<SLAM::Geometry::Frontier *> &frontiers);

        /**
          * A <b>synchronized</b> method to enable/disable the exploration module.
          * @param activate <b>true</b> if you want to use the module; <b>false</b> otherwise.
          */
        void changeStatus(bool activate);

        /*
         * see QThread
         */
        virtual void start();

        /**
          * Slot used to capture the wireless message that contains the signal strenght answer.
          */
        void onUpdateSignalStrength(const Data::Message &msg);

        void onEnableUserCriterionSignal(bool activate, const Data::HighLevelCommand *command);

        void handleBadFrontier(const Data::Pose pose);

        void cleanBadFrontiers();

    public:
        /**
          * Method used to initialize some data of the module.
          */
        void setStartingValues();

    private slots:
        void onStartTimer();
        void onStopTimer();
        void onTimeout();
        //PRM
        void onPerformActionEM(PathPlanner::AbstractAction* action);
        void onRestartExplorationEM();
        void onFrontierToReachEM(Data::Pose pose);
        //

        QList<SLAM::Geometry::Frontier *> filterFrontiers(QList<SLAM::Geometry::Frontier *> frontiers);

    protected:
        virtual void run();

        virtual bool searchNewFrontiers(const SLAM::Map &map);
        virtual void createEvaluationFunction(EvalType type);
        virtual void updateSignalStrengths();
        virtual const QList<SLAM::Geometry::Frontier *> removeBadFrontiers(const QList<SLAM::Geometry::Frontier* > &frontiers);

        EvaluationRecords * startEvaluation(bool startNewAuction, const QList<SLAM::Geometry::Frontier *> &frontiers);
        bool checkIfAllNan(QHash<uint, double> *signalPowerData);
        EvalType eType;
        bool active;
        uint robotId;
        SLAM::SLAMModule *slamModule;
        QTimer *mapRenewTimer;
        Data::RobotState *robotState;
        SLAM::Map map;
        EvaluationFunction *evalFunction;
        SemanticMapping::SemanticMappingModule *semMapModule;
        Coordination::CoordinationModule *coordModule;
        QMutex *stateMutex;
        double signalStrength;
        double distanceFromBaseStation;
        bool newFrontiersFound, forceFrontier;
        QList<SLAM::Geometry::Point *> * badFrontiersCentroids;
        //PRM
        PRM::PRMFunction *evalFunctionPRM;
        PRM::PathPlannerPRM* plannerPRM;
        //
    };
}
#endif // EXPLORATIONMODULE_H
