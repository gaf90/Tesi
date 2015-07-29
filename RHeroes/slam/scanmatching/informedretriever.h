/*
 * informedretriever.h
 *
 *  Created on: 28/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef INFORMEDRETRIEVER_H_
#define INFORMEDRETRIEVER_H_

#include "retriever.h"

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class InformedRetriever : public Retriever<Derived>
{
public:
    using Retriever<Derived>::derived;

    inline Geometry::UncertainLineSegment querySegment(int index) {
        return Geometry::UncertainLineSegment(
                Retriever<Derived>::querySegment(index),
                querySegmentCovariance(index));
    }

    inline const Eigen::Matrix3d queryPoseCovariance() {
        return derived().queryPoseCovariance();
    }

    inline const Eigen::Matrix4d querySegmentCovariance(int index) {
        return derived().querySegmentCovariance(index);
    }

    inline const Eigen::Matrix4d querySegmentScanCovariance(int index) {
        return derived().querySegmentScanCovariance(index);
    }
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* INFORMEDRETRIEVER_H_ */
