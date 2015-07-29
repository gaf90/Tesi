/*
 * uniformweightingstrategy.h
 *
 *  Created on: 10/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef UNIFORMWEIGHTINGSTRATEGY_H_
#define UNIFORMWEIGHTINGSTRATEGY_H_

#include "weightingstrategy.h"

namespace SLAM {
namespace ScanMatching {

class UniformWeightingStrategy : public WeightingStrategy<UniformWeightingStrategy> {
public:
    template <typename S>
    static inline double angleWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        Q_UNUSED(retr) Q_UNUSED(queryidx) Q_UNUSED(mapidx)
        return 1;
    }

    template <typename S>
    static inline double translationWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        Q_UNUSED(retr) Q_UNUSED(queryidx) Q_UNUSED(mapidx)
        return 1;
    }
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* UNIFORMWEIGHTINGSTRATEGY_H_ */
