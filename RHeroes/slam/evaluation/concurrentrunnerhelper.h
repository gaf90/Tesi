/*
 * concurrentrunnerhelper.h
 *
 *  Created on: 25/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef CONCURRENTRUNNERHELPER_H_
#define CONCURRENTRUNNERHELPER_H_

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCoreApplication>
#include "functionworker.h"

namespace SLAM {
namespace Evaluation {

class ConcurrentRunnerHelper : public QThread {
    Q_OBJECT
public:
    ConcurrentRunnerHelper(int maxthreads, void (*func)(void *), const QList<void *> &args);
    ~ConcurrentRunnerHelper();

    bool running() const;
    bool freeSpace() const;
    void finishedOne();

protected:
    void run();

private:
    QList<FunctionWorker *> workers, toProcess;
    int runCount, maxthreads;
    QMutex mutex, finishmutex;
    QWaitCondition cond;
    QCoreApplication app;
};
} /* namespace Evaluation */
} /* namespace SLAM */

#endif /* CONCURRENTRUNNERHELPER_H_ */
