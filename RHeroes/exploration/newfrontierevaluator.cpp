#include "newfrontierevaluator.h"
#include <QMetaType>

namespace Exploration{

using namespace SLAM;
using namespace SLAM::Geometry;
using namespace Coordination;

NewFrontierEvaluator::NewFrontierEvaluator(
        const Map &map, CoordinationModule *coord, uint robotId, int batteryTime,
        EvaluationFunction *evalFun, QHash<uint, double> *signalPowerData, QObject *parent) :
    QThread(parent), map(map), coord(coord), robotId(robotId), batteryTime(batteryTime), evalFun(evalFun),
    signalPowerData(new QHash<uint, double>(*signalPowerData))
{

    qRegisterMetaType<EvaluationRecords>("EvaluationRecords");
    connect(this, SIGNAL(sigStartNewAuction(EvaluationRecords*)), this->coord, SLOT(startNewAuction(EvaluationRecords *)), Qt::DirectConnection);
}

NewFrontierEvaluator::~NewFrontierEvaluator()
{

}

void NewFrontierEvaluator::onFinished()
{
    delete this;
}

void NewFrontierEvaluator::run()
{

    const QList<Frontier*> frontiers =  map.frontiers();
    EvaluationRecords * evalRec = evalFun->evaluateFrontiers(frontiers, map, batteryTime, *signalPowerData);
    //call Giusto's part, passing evalRec.
    emit sigStartNewAuction(evalRec);    
}

}
