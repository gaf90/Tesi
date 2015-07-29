#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H

#include <QList>
#include "evaluationfunction.h"
#include "criteria/criterion.h"
#include "criteria/weightmatrix.h"

namespace Exploration {
    /**
      * This class implements the MCDM evaluation function
      * to evaluate the utility of the frontiers.
      */
    class MCDMFunction : public EvaluationFunction
    {

    public:
        MCDMFunction(uint robotId);
        virtual ~MCDMFunction();

        virtual double evaluateFrontier(
                const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                int batteryTime, QHash<uint, double> &powerSignalData);

        virtual EvaluationRecords* evaluateFrontiers(const QList<SLAM::Geometry::Frontier *> &frontiers, const SLAM::Map &map,
                                                     int batteryTime, QHash<uint, double> &powerSignalData);

        virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);

    private:

        Criterion * createCriterion(QString name, double weight);        
        QHash<QString, Criterion *> *criteria;
        QList<Criterion *> *activeCriteria;
        WeightMatrix * matrix;


    };
}
#endif // MCDMFUNCTION_H
