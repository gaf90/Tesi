#ifndef PRMFUNCTION_H
#define PRMFUNCTION_H

#include "PRM/aStarPRM/astaralgorithmprm.h"
#include "slam/geometry/frontier.h"
#include "PRM/prmalgorithm.h"
#include "PRM/evaluationPRM/evaluationrecordsprm.h"
#include "prmweightreader.h"
#include "criteriaPRM/criterionprm.h"
#include "testPRM/testevaluationprm.h"
#include "PRM/pathplannerprm.h"
#include <QObject>

namespace PRM{

using namespace SLAM;
using namespace SLAM::Geometry;

class PRMFunction : public EvaluationFunction
{
    friend class TestPRM::TestEvaluationPRM;
public:
    PRMFunction(PRMAlgorithm* prm, PathPlannerPRM* planner, uint robotId);

    virtual EvaluationRecords* evaluateFrontiers(const QList<Frontier*> &f, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData);

    virtual double evaluateFrontier(const Frontier* frontier, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData);

    virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);

private:
    PRMAlgorithm* prmAlgorithm;
    PathPlannerPRM* planner;
    QHash<QString, CriterionPRM*>* criteria;
    QList<CriterionPRM*> *activeCriteria;
    int sameFrontierCounter;
    Frontier* previousFrontier;
    WeightMatrix* matrix;
    QHash<const Frontier, QList<PRMPath> >* pathsList;
    QHash<Frontier, PRMPath >* bestPathList;

    CriterionPRM* createCriterion(QString name, double weight);
};

}
#endif // PRMFUNCTION_H
