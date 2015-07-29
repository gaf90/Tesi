/*
 * fullyinformedretriever.h
 *
 *  Created on: 28/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef FULLYINFORMEDRETRIEVER_H_
#define FULLYINFORMEDRETRIEVER_H_

#include "informedretriever.h"

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class FullyInformedRetriever : public InformedRetriever<Derived>
{
public:
    using InformedRetriever<Derived>::derived;

    inline Geometry::UncertainLineSegment mapSegment(int index) {
        return Geometry::UncertainLineSegment(
                Retriever<Derived>::mapSegment(index),
                mapSegmentCovariance(index));
    }

    inline const Eigen::Matrix4d mapSegmentCovariance(int index) {
        return derived().mapSegmentCovariance(index);
    }

    inline const Eigen::Matrix4d queryMapSegmentsJointCovariance(int queryidx, int mapidx) {
        return derived().queryMapSegmentsJointCovariance(queryidx, mapidx);
    }

    inline const Eigen::Matrix4d mapSegmentScanCovariance(int index) {
        return derived().mapSegmentScanCovariance(index);
    }

    inline Geometry::Rototranslation mapPose(int index) {
        return derived().mapPose(index);
    }

    inline Eigen::Matrix3d mapPoseCovariance(int index) {
        return derived().mapPoseCovariance(index);
    }

    inline Eigen::Matrix3d queryPoseMapPoseJointCovariance(int mapidx) {
        return derived().queryPoseMapPoseJointCovariance(mapidx);
    }
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* FULLYINFORMEDRETRIEVER_H_ */
