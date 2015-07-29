/*
 * functionworker.cpp
 *
 *  Created on: 25/mar/2013
 *      Author: Mladen Mazuran
 */

#include "functionworker.h"
#include "concurrentrunnerhelper.h"

namespace SLAM {
namespace Evaluation {

FunctionWorker::FunctionWorker(void (*func)(void *), void *arg, ConcurrentRunnerHelper *helper) :
    helper(helper), func(func), arg(arg) {
}

void FunctionWorker::run() {
    func(arg);
    helper->finishedOne();
}

} /* namespace Evaluation */
} /* namespace SLAM */
