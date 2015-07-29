#ifndef NEWFRONTIEREVALUATOR_H
#define NEWFRONTIEREVALUATOR_H

#include <QThread>
#include <QHash>
#include "slam/map.h"
#include "coordination/coordinationmodule.h"
#include "evaluationrecords.h"

namespace Exploration {

class NewFrontierEvaluator : public QThread
{
    Q_OBJECT
public:
    explicit NewFrontierEvaluator(const SLAM::Map &map, Coordination::CoordinationModule *coord,
                                  uint robotId, int batteryTime, EvaluationFunction *evalFun,
                                  QHash<uint, double> *signalPowerData,
                                  QObject *parent = 0);
    virtual ~NewFrontierEvaluator();
signals:
    void sigStartNewAuction(EvaluationRecords *evalRec);

public slots:
    void run();

private slots:
    void onFinished();
private:
    const SLAM::Map map;
    Coordination::CoordinationModule *coord;
    uint robotId;
    int batteryTime;
    EvaluationFunction * evalFun;
    QHash<uint, double> * signalPowerData;

    
};
}

#endif // NEWFRONTIEREVALUATOR_H
