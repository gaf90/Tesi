/*
 * functionworker.h
 *
 *  Created on: 25/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef FUNCTIONWORKER_H_
#define FUNCTIONWORKER_H_

#include <QThread>

namespace SLAM {
namespace Evaluation {

class ConcurrentRunnerHelper;

class FunctionWorker : public QThread {
    Q_OBJECT
public:
    FunctionWorker(void (*func)(void *), void *arg, ConcurrentRunnerHelper *helper);

protected:
    void run();

private:
    ConcurrentRunnerHelper *helper;
    void (*func)(void *);
    void *arg;
};


} /* namespace Evaluation */
} /* namespace SLAM */

#endif /* FUNCTIONWORKER_H_ */
