/*
 * associationposecentric.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef ASSOCIATIONPOSECENTRIC_H_
#define ASSOCIATIONPOSECENTRIC_H_

#include "associationbase.h"
#include "fullyinformedretriever.h"
#include "slam/utilities.h"
#include "slam/support/inverses.h"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace SLAM {
namespace ScanMatching {

class AssociationPoseCentric : public AssociationBase<AssociationPoseCentric>
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

public:
    static Eigen::Matrix3d estimateEmulatedCovariance(
            const Geometry::Rototranslation &refpose,
            const Geometry::LineSegment &s, const Eigen::Matrix4d &scov);
    static double distance(
            const Geometry::Rototranslation &queryPose,
            const Geometry::LineSegment &query,
            const Geometry::LineSegment &map,
            const Eigen::Matrix3d &relativeCovariance);
};

template <typename S>
inline double AssociationPoseCentric::distance(
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

inline Eigen::Matrix3d AssociationPoseCentric::estimateEmulatedCovariance(
        const Geometry::Rototranslation &refpose,
        const Geometry::LineSegment &s, const Eigen::Matrix4d &scov)
{
    const double alpha = s.alpha(), rho = s.rho();

    /* Marginal variances at CRU */
    const double salpha = scov(0,0);
    const double srho = scov(1,1) - square(scov(0,1)) / scov(0,0);
    const double spsi = scov(2,2) + scov(3,3);

    const Geometry::Point cru = Geometry::UncertainLineSegment::cru(alpha, rho, scov.block<2,2>(0,0));

    Geometry::Rototranslation p(cru.x(), cru.y(), alpha);

    const double c2 = square(p.cosAngle());
    const double s2 = square(p.sinAngle());
    const double cs = p.cosAngle() * p.sinAngle();
    Eigen::Matrix3d pcov;
    pcov <<
            spsi * c2 + srho * s2, (spsi - srho) * cs, 0,
            (spsi - srho) * cs, srho * c2 + spsi * s2, 0,
            0, 0, salpha;

    Eigen::Matrix3d invJ;
    invJ <<
            1, 0, p.ty() * refpose.cosAngle() + p.tx() * refpose.sinAngle(),
            0, 1, p.ty() * refpose.sinAngle() - p.tx() * refpose.cosAngle(),
            0, 0, 1;
    return invJ * pcov * invJ.transpose();
#if 0
    const double alpha = s.alpha(), rho = s.rho();
    const Geometry::Point cru = Geometry::UncertainLineSegment::cru(alpha, rho, scov.block<2,2>(0,0));
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    const double drho = cru.x() * ca + cru.y() * sa;
    const double dpsi = cru.y() * ca - cru.x() * sa;
    Eigen::Matrix<double, 3, 4> pinv;
    pinv <<
            -dpsi * ca - drho * sa, ca, -sa / 2, -sa / 2,
             drho * ca - dpsi * sa, sa,  ca / 2,  ca / 2,
             1, 0, 0, 0;
    return pinv * scov * pinv.transpose();
#endif

}

inline double AssociationPoseCentric::distance(
        const Geometry::Rototranslation &queryPose,
        const Geometry::LineSegment &query,
        const Geometry::LineSegment &map,
        const Eigen::Matrix3d &relativeCovariance)
{
    Eigen::Vector4d queryvec = (queryPose.inverse() * query).vector(), mapvec = map.vector();

    if(std::abs(wrap(queryPose.angle() + queryvec[0] - mapvec[0])) > M_PI_2) {
        mapvec[0] = wrap(mapvec[0] + M_PI);
        mapvec.block<3,1>(1,0) *= -1;
    }

    const double c = std::cos(mapvec[0]), s = std::sin(mapvec[0]);
    Eigen::Vector3d diffb = queryPose.vectorForm() - Eigen::Vector3d(
            (mapvec[1] - queryvec[1]) * c, (mapvec[1] - queryvec[1]) * s,
            mapvec[0] - queryvec[0]);
    Eigen::Vector3d diffa(s, -c, 0);
    diffb[2] = wrap(diffb[2]);

    const double psimaxq = max(queryvec[2], queryvec[3]);
    const double psiminq = min(queryvec[2], queryvec[3]);
    const double psimaxm = max(mapvec[2],   mapvec[3]);
    const double psiminm = min(mapvec[2],   mapvec[3]);
    const double tmin    = psiminq - psimaxm;
    const double tmax    = psimaxq - psiminm;

    Eigen::LLT<Eigen::Matrix3d> llt(relativeCovariance);

    const double t =
            static_cast<double>(diffb.transpose() * llt.solve(diffa)) /
            static_cast<double>(diffa.transpose() * llt.solve(diffa));
    if(t >= tmin && t <= tmax) {
        Eigen::Vector3d diff = diffb - diffa * t;
        return diff.transpose() * llt.solve(diff);
    } else {
        Eigen::Vector3d diff1 = diffb - diffa * tmin;
        Eigen::Vector3d diff2 = diffb - diffa * tmax;

        return min(diff1.transpose() * llt.solve(diff1), diff2.transpose() * llt.solve(diff2));
    }
}

/* Stripped down version of the FullyInformedRetriever one */
template <typename S>
inline double AssociationPoseCentric::distance(
        InformedRetriever<S> &retr, int queryidx, int mapidx)
{
    S &r = retr.derived();

    Geometry::Rototranslation queryPose = r.queryPose();
    Geometry::LineSegment query = r.querySegment(queryidx);
    Geometry::LineSegment map   = r.mapSegment(mapidx);

    Eigen::Matrix3d relativeCovariance = r.queryPoseCovariance() +
            estimateEmulatedCovariance(queryPose, query, r.querySegmentScanCovariance(queryidx));

    //ldbg << ">>" << relativeCovariance << endl;

    //ldbg << "querypose: " << queryPose.vectorForm() << endl;
    //ldbg << "querycov: " << r.querySegmentScanCovariance(queryidx) << endl;
    //ldbg << "posecov: " << r.queryPoseCovariance() << endl;
    //ldbg << "emulated: " << estimateEmulatedCovariance(queryPose, query, r.querySegmentScanCovariance(queryidx)) << endl;

    Geometry::UncertainLineSegment uq(query, 1e-6 * Eigen::Matrix4d::Identity());
    Geometry::UncertainLineSegment um(map,   1e-6 * Eigen::Matrix4d::Identity());
    double d = distance(queryPose, query, map, relativeCovariance);
    //ldbg << "Graphics[{" << uq << "," << um << "}, (*PlotRange -> {{-10, 10}, {0, 20}},*) " <<
    //        "Frame -> True, ImageSize -> 640]" << endl << "Print[" << d << "];" << endl;
    return d;
}

template <typename S>
inline double AssociationPoseCentric::distance(
        FullyInformedRetriever<S> &retr, int queryidx, int mapidx)
{
    S &r = retr.derived();

    Geometry::Rototranslation queryPose = r.queryPose();
    Geometry::Rototranslation mapPose = r.mapPose(mapidx);
    Geometry::LineSegment query = r.querySegment(queryidx);
    Geometry::LineSegment map   = r.mapSegment(mapidx);

    Eigen::Matrix3d relativeCovariance =
            estimateEmulatedCovariance(queryPose, query, r.querySegmentScanCovariance(queryidx)) +
            estimateEmulatedCovariance(mapPose, map, r.mapSegmentScanCovariance(queryidx)) +
            r.queryPoseCovariance() + r.mapPoseCovariance(mapidx) -
            r.queryPoseMapPoseJointCovariance(mapidx) -
            r.queryPoseMapPoseJointCovariance(mapidx).transpose();


    return distance(queryPose, query, map, relativeCovariance);
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* ASSOCIATIONPOSECENTRIC_H_ */
