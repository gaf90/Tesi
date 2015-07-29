/*
 * associationprobabilistic.h
 *
 *  Created on: 28/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef ASSOCIATIONPROBABILISTIC_H_
#define ASSOCIATIONPROBABILISTIC_H_

#include "associationbase.h"
#include "fullyinformedretriever.h"
#include "slam/utilities.h"
#include "slam/support/inverses.h"
#include <Eigen/Core>
#include <Eigen/LU>

namespace SLAM {
namespace ScanMatching {

class AssociationProbabilistic : public AssociationBase<AssociationProbabilistic>
{
public:
    template <typename S>
    static double distance(Retriever<S> &retr, int queryidx, int mapidx);

    template <typename S>
    static double distance(InformedRetriever<S> &retr, int queryidx, int mapidx);

    template <typename S>
    static double distance(FullyInformedRetriever<S> &retr, int queryidx, int mapidx);

    static const double &lookupThreshold;
    static const double &broadLookupThreshold;

private:
    /* Provide a short-hand for the index names */
    static const int alphaIndex = Geometry::UncertainLineSegment::AlphaVectorIndex;
    static const int rhoIndex   = Geometry::UncertainLineSegment::RhoVectorIndex;
    static const int psi1Index  = Geometry::UncertainLineSegment::Psi1VectorIndex;
    static const int psi2Index  = Geometry::UncertainLineSegment::Psi2VectorIndex;
};

template <typename S>
inline double AssociationProbabilistic::distance(
        Retriever<S> &retr, int queryidx, int mapidx)
{
    /*
        Disable compilation for non-informed retrievers. The method declaration is required in
        order to avoid an infinite recursion due to static polymorphism.
    */
    Q_UNUSED(retr) Q_UNUSED(queryidx) Q_UNUSED(mapidx)
    static_assert(delayed_false(S));
    return 0;
}

/* Stripped down version of the FullyInformedRetriever one */
template <typename S>
inline double AssociationProbabilistic::distance(
        InformedRetriever<S> &retr, int queryidx, int mapidx)
{
    /* Retrieve the query line segment covariance */
    Eigen::Matrix4d queryCov = retr.querySegmentCovariance(queryidx);

    /* Retrieve the two line segments, adding the full covariance */
    const Geometry::UncertainLineSegment query(retr.querySegment(queryidx), queryCov);
    const Geometry::LineSegment map = retr.mapSegment(mapidx);

    /*
        Calculate the point at which to put the new reference frame, in order to reduce the lever
        arm effect, which is the center of rotational uncertainty of the query segment
    */
    const Geometry::Point frameCenter = query.cru();
    /* Build the corresponding translation (no rotation involved) */
    const Geometry::Rototranslation rt(-frameCenter.x(), -frameCenter.y(), 0);

    /* Express the two line segments in the new reference frame */
    const Geometry::UncertainLineSegment queryRT = rt * query;
    Geometry::UncertainLineSegment mapRT(rt * map, Eigen::Matrix4d::Identity());

    /*
        Calculate the jacobian that maps the line segment parameters from the old to the
        new reference frame
    */
    Eigen::Matrix4d Hs = query.transformationJacobian(rt);

    /*
        If the absolute value of the difference between the angles is greater than pi/2, then we
        need to change the orientation of one of them (do it on the second in order to avoid
        readjusting the jacobian)
    */
    if(std::abs(wrap(queryRT.alpha() - mapRT.alpha())) > M_PI_2) {
        Geometry::Rototranslation orient(mapRT.centroid(), M_PI);
        mapRT = Geometry::UncertainLineSegment(orient * rt * map, Eigen::Matrix4d::Identity());
    }

    /*
        Project the query segment end points on the psi frame of the map segment, and retrieve the
        psi coordinates of the end points of the map segment
    */
    double psi1q = mapRT.psiProjection(queryRT.p1());
    double psi2q = mapRT.psiProjection(queryRT.p2());
    double psi1m = mapRT.psi1();
    double psi2m = mapRT.psi2();

    /*
        We expect psi1q to be smaller than psi2q, if it's not swap them and re-adjust the
        covariance
    */
    if(psi1q > psi2q) {
        std::swap(psi1q, psi2q);
        queryCov.col(psi1Index).swap(queryCov.col(psi2Index));
        queryCov.row(psi1Index).swap(queryCov.row(psi2Index));
    }

    /* The same, but for psi1m and psi2m */
    if(psi1m > psi2m) {
        std::swap(psi1m, psi2m);
    }

    /* Lengths of the (projected in the query case) line segments */
    const double lq    = psi2q - psi1q;
    const double lm    = psi2m - psi1m;

    /* Line segments' center values */
    const double psicq = (psi1q + psi2q) / 2;
    const double psicm = (psi1m + psi2m) / 2;

    /* Maximum distance between the center values that would result in any overlap */
    const double delta = (lq + lm) / 2;

    /* Distance between the center values */
    const double dpsic = psicq - psicm;

    /* Update the query segment covariance in order to take into account the rototranslation */
    queryCov = (Hs * queryCov * Hs.transpose()).eval();

    /* Infinite line parameters' difference */
    Eigen::Vector2d lineDiff = queryRT.lineVector() - mapRT.lineVector();
    lineDiff[alphaIndex] = wrap(lineDiff[alphaIndex]);

    /* Piecewise continuous pseudo-Mahalanobis case selection */
    if(std::abs(dpsic) <= delta || lq < 1e-2) {
        /* The overlap component is zero */
        return lineDiff.transpose() * queryCov.block<2,2>(0,0).inverse() * lineDiff;
    } else {
        /* Overlap distance */
        double doverlap;

        /* Full jacobian, including overlap distance*/
        Eigen::Matrix<double, 3, 4> H;

        const double ca = std::cos(lineDiff[alphaIndex]), sa = std::sin(lineDiff[alphaIndex]);

        if(dpsic > delta) {
            const double d1 = queryRT.rho() * ca - psi1q * sa;
            doverlap = dpsic - delta; // psi1q - psi2m
            H <<
                     1,  0,  0,  0,
                     0,  1,  0,  0,
                    d1, sa, ca,  0;
        } else {
            const double d2 = queryRT.rho() * ca - psi2q * sa;
            doverlap = dpsic + delta; // psi2q - psi1m
            H <<
                     1,  0,  0,  0,
                     0,  1,  0,  0,
                    d2, sa,  0, ca;
        }

        //ldbg << lq << endl;

        const Eigen::Matrix3d mahalanobisCov = H * queryCov * H.transpose();
        const Eigen::Vector3d eps(lineDiff[0], lineDiff[1], doverlap);

        //ldbg << mahalanobisCov << endl;

        return eps.transpose() * Support::pseudoInverse(mahalanobisCov) * eps;
    }
}

template <typename S>
inline double AssociationProbabilistic::distance(
        FullyInformedRetriever<S> &retr, int queryidx, int mapidx)
{

    S &r = retr.derived();

    /* Retrieve the line segment / pose covariances */
    Eigen::Matrix3d relativePoseCov = r.queryPoseCovariance() + r.mapPoseCovariance(mapidx) -
            r.queryPoseMapPoseJointCovariance(mapidx) -
            r.queryPoseMapPoseJointCovariance(mapidx).transpose();

    /* Retrieve the two line segments and add their full scan covariances */
    Geometry::UncertainLineSegment query(
            r.querySegment(queryidx), r.querySegmentScanCovariance(queryidx));
    Geometry::UncertainLineSegment map(
            r.mapSegment(mapidx), r.mapSegmentScanCovariance(mapidx));

    /*
        Express line segments so that the map segment has (alpha, rho) = (0, 0) and add relative
        pose uncertainty to the query segment
    */
    Geometry::Rototranslation remap(-map.rho(), 0, -map.alpha());
    query = (remap * Geometry::UncertainRototranslation(r.queryPose(), relativePoseCov) *
                r.queryPose().inverse()) * query;
    map = remap * map;

    /*
        Calculate the point at which to put the new reference frame, in order to reduce the lever
        arm effect.
    */
    Eigen::Matrix2d relativeLineCov = query.lineCovariance() + map.lineCovariance();
    const Geometry::Point frameCenter = Geometry::UncertainLineSegment::cru(
            query.alpha(), query.rho(), relativeLineCov);
    const Geometry::Rototranslation rt(-frameCenter.x(), -frameCenter.y(), 0);

    /* Express the two line segments in the new reference frame */
    Geometry::UncertainLineSegment queryRT = rt * query;
    Geometry::UncertainLineSegment mapRT = rt * map;

    /*
        If the absolute value of the difference between the angles is greater than pi/2, then we
        need to change the orientation of one of them
    */
    if(std::abs(wrap(queryRT.alpha() - mapRT.alpha())) > M_PI_2) {
        queryRT = Geometry::Rototranslation(queryRT.centroid(), M_PI) * queryRT;
    }

    /*
        Project the query segment end points on the psi frame of the map segment, and retrieve the
        psi coordinates of the end points of the map segment
    */
    double psi1q = mapRT.psiProjection(queryRT.p1());
    double psi2q = mapRT.psiProjection(queryRT.p2());
    double psi1m = mapRT.psi1();
    double psi2m = mapRT.psi2();

    /*
        We expect psi1q to be smaller than psi2q, if it's not swap them and re-adjust the
        covariances
    */
    if(psi1q > psi2q) {
        std::swap(psi1q, psi2q);
        queryRT.swapEndpoints();
    }

    /* The same, but for psi1m and psi2m */
    if(psi1m > psi2m) {
        std::swap(psi1m, psi2m);
        mapRT.swapEndpoints();
    }

    /* Lengths of the (projected in the query case) line segments */
    const double lq    = psi2q - psi1q;
    const double lm    = psi2m - psi1m;

    /* Line segments' center values */
    const double psicq = (psi1q + psi2q) / 2;
    const double psicm = (psi1m + psi2m) / 2;

    /* Maximum distance between the center values that would result in any overlap */
    const double delta = (lq + lm) / 2;

    /* Distance between the center values */
    const double dpsic = psicq - psicm;

    /* Infinite line parameters' difference */
    Eigen::Vector2d lineDiff = queryRT.lineVector() - mapRT.lineVector();
    lineDiff[alphaIndex] = wrap(lineDiff[alphaIndex]);
    const Eigen::Matrix2d lineDiffCov =
            queryRT.covariance().block<2,2>(0,0) + mapRT.covariance().block<2,2>(0,0);

    /* Piecewise continuous pseudo-Mahalanobis case selection */
    if(std::abs(dpsic) <= delta || lq < 1e-2) {
        /* The overlap component is zero */
        return lineDiff.transpose() * lineDiffCov.inverse() * lineDiff;
    } else {
        /* Overlap distance */
        double doverlap;

        /* Full jacobian, including overlap distance*/
        Eigen::RowVector4d H1, H2;

        const double ca = std::cos(lineDiff[alphaIndex]), sa = std::sin(lineDiff[alphaIndex]);

        if(dpsic > delta) {
            const double d1 = queryRT.rho() * ca - psi1q * sa;
            doverlap = dpsic - delta; // psi1q - psi2m
            H1 <<  d1,  sa,  ca,   0,
            H2 << -d1,   0,   0,  -1;
        } else {
            const double d2 = queryRT.rho() * ca - psi2q * sa;
            doverlap = dpsic + delta; // psi2q - psi1m
            H1 <<  d2,  sa,   0,  ca;
            H2 << -d2,   0,  -1,   0;
        }

        const Eigen::RowVector4d H1Sq = H1 * queryRT.covariance();
        const Eigen::RowVector4d H2Sm = H2 * mapRT.covariance();
        const Eigen::RowVector2d offdiagonal = H1Sq.block<1,2>(0,0) - H2Sm.block<1,2>(0,0);

        const Eigen::Vector3d eps(lineDiff[0], lineDiff[1], doverlap);
        Eigen::Matrix3d mahalanobisCov;

        mahalanobisCov <<
                lineDiffCov, offdiagonal.transpose(),
                offdiagonal, H1Sq * H1.transpose() + H2Sm * H2.transpose();

        return eps.transpose() * Support::pseudoInverse(mahalanobisCov) * eps;
    }
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* ASSOCIATIONPROBABILISTIC_H_ */
