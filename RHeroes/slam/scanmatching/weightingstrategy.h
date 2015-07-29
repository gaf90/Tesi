/*
 * weightingstrategy.h
 *
 *  Created on: 10/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef WEIGHTINGSTRATEGY_H_
#define WEIGHTINGSTRATEGY_H_

#include "retriever.h"

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class WeightingStrategy {
public:
    template <typename S>
    static inline double angleWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        return Derived::angleWeight(retr.derived(), queryidx, mapidx);
    }

    template <typename S>
    static inline double translationWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        return Derived::translationWeight(retr.derived(), queryidx, mapidx);
    }
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* WEIGHTINGSTRATEGY_H_ */
