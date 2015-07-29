/*
 * concurrentrunnerhelper.cpp
 *
 *  Created on: 25/mar/2013
 *      Author: Mladen Mazuran
 */

#include "concurrentrunnerhelper.h"
#include "shared/logger.h"
#include <QTimer>

namespace SLAM {
namespace Evaluation {

static int fake_argc = 0;
static char **fake_argv = NULL;

ConcurrentRunnerHelper::ConcurrentRunnerHelper(
        int maxthreads, void (*func)(void *), const QList<void *> &args) :
    runCount(0), maxthreads(maxthreads), app(fake_argc, fake_argv)
{
    for(int i = 0; i < args.size(); i++) {
        workers.append(new FunctionWorker(func, args[i], this));
        toProcess.append(workers.last());
    }
    run();
    start();
    app.exec();
}

ConcurrentRunnerHelper::~ConcurrentRunnerHelper()
{
    qDeleteAll(workers);
}

void ConcurrentRunnerHelper::run() {
    mutex.lock();
    moveToThread(app.thread());
    while(runCount > 0 || toProcess.size() > 0) {
        if(toProcess.size() > 0 && runCount < maxthreads) {
            finishmutex.lock();
            FunctionWorker *worker = toProcess.first();
            toProcess.removeFirst();
            runCount++;
            worker->start();
            finishmutex.unlock();
        } else {
            cond.wait(&mutex);
        }
    }
    mutex.unlock();
    QThread::msleep(100);
    app.quit();
}

void ConcurrentRunnerHelper::finishedOne() {
    finishmutex.lock();
    runCount--;
    cond.wakeAll();
    finishmutex.unlock();
}

} /* namespace Evaluation */
} /* namespace SLAM */
