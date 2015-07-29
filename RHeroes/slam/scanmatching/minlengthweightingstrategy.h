/*
 * minlengthweightingstrategy.h
 *
 *  Created on: 10/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef MINLENGTHWEIGHTINGSTRATEGY_H_
#define MINLENGTHWEIGHTINGSTRATEGY_H_

#include "weightingstrategy.h"

namespace SLAM {
namespace ScanMatching {

class MinLengthWeightingStrategy : public WeightingStrategy<MinLengthWeightingStrategy> {
public:
    template <typename S>
    static inline double angleWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        return std::min(retr.querySegment(queryidx).length(), retr.mapSegment(mapidx).length());
    }

    template <typename S>
    static inline double translationWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        return std::min(retr.querySegment(queryidx).length(), retr.mapSegment(mapidx).length());
    }
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* MINLENGTHWEIGHTINGSTRATEGY_H_ */
