/*
 * mlweightingstrategy.h
 *
 *  Created on: 10/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef MLWEIGHTINGSTRATEGY_H_
#define MLWEIGHTINGSTRATEGY_H_

#include "fullyinformedretriever.h"
#include "weightingstrategy.h"
#include "slam/support/assert.h"

namespace SLAM {

namespace ScanMatching {

/* (Approximate) Maximum likelihood weighting strategy */
class MLWeightingStrategy : public WeightingStrategy<MLWeightingStrategy> {
public:
    template <typename S>
    static inline double angleWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        Q_UNUSED(retr) Q_UNUSED(queryidx) Q_UNUSED(mapidx)
        /* Disable compilation for uninformed retrievers */
        static_assert(delayed_false(S));
        return 0;
    }

    template <typename S>
    static inline double translationWeight(Retriever<S> &retr, int queryidx, int mapidx) {
        Q_UNUSED(retr) Q_UNUSED(queryidx) Q_UNUSED(mapidx)
        /* Disable compilation for uninformed retrievers */
        static_assert(delayed_false(S));
        return 0;
    }

    template <typename S>
    static inline double angleWeight(InformedRetriever<S> &retr, int queryidx, int mapidx) {
        Q_UNUSED(mapidx)
        return retr.derived().querySegmentCovariance(queryidx)(0,0);
    }

    template <typename S>
    static inline double translationWeight(InformedRetriever<S> &retr, int queryidx, int mapidx) {
        const Geometry::LineSegment map    = retr.mapSegment(mapidx);
        const Eigen::Vector2d queryVec     = retr.querySegment(queryidx).lineVector();
        const Eigen::Vector2d mapVec       = map.lineVector();
        Eigen::Matrix<double, 1, 4> J;
        J << (map.psi1() + map.psi2()) * std::cos(queryVec[alphaIndex] - mapVec[alphaIndex]) / 2 -
                mapVec[rhoIndex] * std::sin(queryVec[alphaIndex] - mapVec[alphaIndex]), -1, 0, 0;
        return J * retr.querySegmentCovariance(queryidx) * J.transpose();
    }

    template <typename S>
    static inline double angleWeight(FullyInformedRetriever<S> &retr, int queryidx, int mapidx) {
        return retr.querySegmentCovariance(queryidx)(0,0) +
                retr.mapSegmentCovariance(queryidx)(0,0) -
                2 * retr.queryMapSegmentsJointCovariance(queryidx, mapidx)(0,0);
    }

    template <typename S>
    static inline double translationWeight(
            FullyInformedRetriever<S> &retr, int queryidx, int mapidx) {
        const Geometry::LineSegment map    = retr.mapSegment(mapidx);
        const Eigen::Vector2d queryVec     = retr.querySegment(queryidx).lineVector();
        const Eigen::Vector2d mapVec       = map.lineVector();
        const double ca = std::cos(queryVec[alphaIndex] - mapVec[alphaIndex]);
        const double sa = std::sin(queryVec[alphaIndex] - mapVec[alphaIndex]);
        const double d1 = (map.psi1() + map.psi2()) * ca / 2 - mapVec[rhoIndex] * sa;
        Eigen::Matrix<double, 1, 8> J;
        Eigen::Matrix<double, 8, 8> fullCov;

        J <<
                d1, -1, 0, 0, -d1, ca, sa / 2, sa / 2;
        fullCov <<
                retr.querySegmentCovariance(queryidx),
                retr.queryMapSegmentsJointCovariance(queryidx, mapidx),
                retr.queryMapSegmentsJointCovariance(queryidx, mapidx).transpose(),
                retr.mapSegmentCovariance(mapidx);
        return J * fullCov * J.transpose();
    }

private:
    static const int alphaIndex = Geometry::LineSegment::AlphaVectorIndex;
    static const int rhoIndex   = Geometry::LineSegment::RhoVectorIndex;
};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* MLWEIGHTINGSTRATEGY_H_ */
