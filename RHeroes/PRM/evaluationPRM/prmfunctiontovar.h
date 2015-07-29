#ifndef PRMFUNCTIONTOVAR_H
#define PRMFUNCTIONTOVAR_H

#include "PRM/aStarPRM/astaralgorithmprm.h"
#include "slam/geometry/frontier.h"
#include "PRM/prmalgorithm.h"
#include "PRM/evaluationPRM/evaluationrecordsprm.h"
#include "testPRM/testevaluationprm.h"
#include "PRM/pathplannerprm.h"
#include <QObject>

namespace PRM{

using namespace SLAM;
using namespace SLAM::Geometry;

class PRMFunctionTovar : public EvaluationFunction
{
    friend class TestPRM::TestEvaluationPRM;
public:
    PRMFunctionTovar(PRMAlgorithm* prm, PathPlannerPRM* planner, uint robotId);

    virtual EvaluationRecords* evaluateFrontiers(const QList<Frontier*> &f, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData);

    virtual double evaluateFrontier(const Frontier* frontier, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData);

    virtual void onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command);

private:
    PRMAlgorithm* prmAlgorithm;
    PathPlannerPRM* planner;
    int sameFrontierCounter;
    Frontier* previousFrontier;
    QHash<const Frontier, QList<PRMPath> >* pathsList;
    QHash<Frontier, PRMPath >* bestPathList;
    QHash<Frontier, QList<double> >* frontierValue;

    void normalizeTovar();
};

}


#endif // PRMFUNCTIONTOVAR_H
