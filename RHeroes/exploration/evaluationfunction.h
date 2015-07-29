#ifndef EVALUATIONFUNCTION_H
#define EVALUATIONFUNCTION_H

#include "slam/geometry/frontier.h"
#include "evaluationrecords.h"
#include "data/message.h"
#include "slam/map.h"
#include "data/highlevelcommand.h"
#include <QObject>
#include <QHash>
#include <QUuid>

namespace Exploration{

    enum EvalType{

        MCDM,
        AOJRF,
        BasicSemantic,
        Dummy,
        //PRM
        PRM,
        Tovar
        //
    };

    /**
      * This class is the base class for all the evaluation
      * functions we will implement.
      */
    class EvaluationFunction
    {
    public:
        /**
          * Constructor of the base class
          * @param robotId the robot identifier
          */
        EvaluationFunction(uint robotId);
        virtual ~EvaluationFunction();

        /**
          * Method to compute the evaluation of a single frontier.
          * @param frontier the frontier to evaluate
          * @param map the map to use to evaluate the frontier
          * @return the evaluation of the frontier
          */
        virtual double evaluateFrontier(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                                        int batteryTime, QHash<uint, double> &powerSignalData) = 0;
        /**
          * Method to compute the evaluation of a list of frontiers.
          * @param frontiers the list of frontiers to evaluate
          * @param map the map to use to evaluate the frontier
          * @return an EvaluationRecords that contains the evaluation of all the frontiers
          */
        virtual EvaluationRecords* evaluateFrontiers(const QList<SLAM::Geometry::Frontier *> &frontiers, const SLAM::Map &map,
                                                     int batteryTime, QHash<uint, double> &powerSignalData);

        /**
          * An utility function to compute the information gain of a single frontier
          * @param frontier the frontier for which we need to compute the infromation gain
          * @param map the map in which the frontier is contained
          * @return the information gain of the frontier
          */
        virtual double computeInformationGain(const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map);
        /**
          * An utility function to compute the distance between a robot and a frontier
          * @param frontier the frontier for which we need to compute the distance
          * @param map the map in which the frontier is contained
          * @param robot the identifier of the robot
          * @return the distance between the robot and the frontier
          */
        virtual double computeDistanceFromOtherRobots(const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map, const uint robot);

        /**
          * An utility function to compute the transmission probability between a robot and a frontier, given the powerData matrix
          * @param frontier the frontier for which we need to compute the distance
          * @param map the map in which the frontier is contained
          * @param powerData a list that contains, for each other robot, the signal power between them and this robot.
          * @return the probability that the robot will continue to communicate with other robots if it reach the frontier.
          */
        virtual double computeTransmissionProbability(
                const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map, const QHash<uint, double> &powerData);

        virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command) = 0;

    protected:
        uint robotId;

    };
}

#endif // EVALUATIONFUNCTION_H
