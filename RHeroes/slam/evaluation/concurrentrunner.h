/*
 * concurrentrunner.h
 *
 *  Created on: 25/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef CONCURRENTRUNNER_H_
#define CONCURRENTRUNNER_H_

#include "concurrentrunnerhelper.h"
#include "slam/support/fforeach.h"

namespace SLAM {
namespace Evaluation {

namespace __internal {

template <typename T, void (*F)(T&)>
void functionWrapper(void *arg) {
    F(*reinterpret_cast<T *>(arg));
}

} /* namespace __internal */

template <typename T, void (*F)(T&)>
class ConcurrentRunner {
public:
    ConcurrentRunner(int maxthreads, QList<T> &args) :
        args(args), helper(maxthreads, __internal::functionWrapper<T, F>, buildArguments()) {
    }

    void start() {
        helper.start();
    }

private:
    QList<void *> buildArguments() const {
        QList<void *> ret;
        fforeach(T &arg, args) {
            ret.append(reinterpret_cast<void *>(&arg));
        }
        return ret;
    }

private:
    QList<T> &args;
    ConcurrentRunnerHelper helper;
};

} /* namespace Evaluation */
} /* namespace SLAM */

#endif /* CONCURRENTRUNNER_H_ */
