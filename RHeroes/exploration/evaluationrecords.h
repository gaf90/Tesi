#ifndef EVALUATIONRECORDS_H
#define EVALUATIONRECORDS_H

#include <QHash>
#include "slam/geometry/frontier.h"
#include "data/serializable.h"

namespace Exploration {
    /**
      * Data structure that contains, for each frontier, the utility
      * for the robot to reach the frontier.
      */
    class EvaluationRecords : public Data::Serializable
    {
    public:
        /**
          * Constructor
          * @param robotId the robot identifier
          */
        EvaluationRecords(uint robotId);
        /**
          * Constructor
          */
        EvaluationRecords();
        /**
          * Destructor.
          * The class destroy the frontiers. Be sure to create a copy of the frontiers.
          */
        virtual ~EvaluationRecords();

        /**
          * This method insert an evaluation for the frontier. The method does not make
          * a copy of the frontier, so be sure to create a copy from outside the class and pass
          * the copy of the frontier as first parameter.
          * @param frontier the frontier
          * @param value the utility of the frontier
          */
        void putEvaluation(SLAM::Geometry::Frontier frontier, double value);
        /**
          * Method that return the utility value of the queried frontier
          * @param frontier the frontier for which we need the evaluation
          * @return the evaluation of the frontier.
          */
        double getEvaluation(SLAM::Geometry::Frontier frontier);
        /**
          * Method that return all the evaluation stored in this object.
          * Note that this method does not create a copy of the evaluation so,
          * when you delete this, you will delete also the evaluations.
          * @return the evaluations of all the frontiers.
          */
        QHash<SLAM::Geometry::Frontier, double> * getEvaluations();
        /**
          * Method that return all the evaluation stored in this object.
          * Note that this method does not create a copy of the evaluation so,
          * when you delete this, you will delete also the evaluations.
          * @return the evaluations of all the frontiers, identified by the hash code
          */
        QHash<uint, double> getEvaluationFrontiersBids();
        /**
          * Method that allow to query if a frontier is into the evaluation record
          * @param frontier the frontier to query
          * @return <b>true</b> if the frontier is contained into the object; <b>false</b> otherwise.
          */

        bool contains(SLAM::Geometry::Frontier frontier);
        /**
          * Method that return the number of evaluated frontiers
          * @return the number of evaluations
          */
        int size();
        /**
          * Method that returns the list of all the frontiers that are evaluated.
          * Note that this method creates a copy of the frontiers, so you must delete the list
          * and the frontiers when you do not need them.
          * @return the list of the evaluated frontiers.
          */
        QList<SLAM::Geometry::Frontier> *getEvaluatedFrontiers();
        /**
          * Method that remove a frontier from the evaluation record.
          * @param frontier the frontier to remove.
          */
        void removeFrontier(SLAM::Geometry::Frontier frontier);

        /**
          * Getter for the robot id.
          * @return the robot identifier
          */
        uint getRobotId();
        /**
          * Setter for the robot id
          * @param the robot identifier to set.
          */
        void setRobotId(uint robotId);

        void normalize();



        virtual void serializeTo(QDataStream &stream) const;
        virtual void deserializeFrom(QDataStream &stream);

    protected:
        QHash<SLAM::Geometry::Frontier, double> *evaluations;
        uint robotId;
        double maxValue, minValue;
    };
}

#endif // EVALUATIONRECORDS_H
