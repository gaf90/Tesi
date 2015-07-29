/*
 * decoupledestimator.h
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef DECOUPLEDESTIMATOR_H_
#define DECOUPLEDESTIMATOR_H_

#include "slam/utilities.h"
#include "slam/geometry/segmentscan.h"
#include "shared/utilities.h"
#include "fullyinformedretriever.h"
#include "minlengthweightingstrategy.h"
#include <cmath>
#include <sstream>
#include "slam/scanmatching/associationamigoni.h"
#ifdef SM_USE_STANDARD_INVERSE
#   include <Eigen/Cholesky>
#else
#   include "slam/support/inverses.h"
#endif

namespace SLAM {
namespace ScanMatching {

template <typename S, typename W = MinLengthWeightingStrategy>
class DecoupledEstimator
{
public:
    void setRetriever(Retriever<S> &retr);
    void setAssociations(QList<int> associations[]);
    void estimate();
    void removeAssociation(int scanidx, int refidx);
    Eigen::Vector3d singleAssociationEstimate(int scanidx, int refidx) const;
    Eigen::Vector3d rototranslation() const;
    Eigen::Vector3d pureRototranslation() const;
    Eigen::Vector2d singleAssociationDeviation(int scanidx, int refidx) const;
    Eigen::Matrix3d covariance();
    bool isCompatible(
            const Eigen::Vector3d &rt, double anglethresh = M_PI / 10, double traslthresh = 0.4);
    static Eigen::Vector2d deviation(const Eigen::Vector3d &query, const Eigen::Vector3d &ref);

private:
    void updateRotationFromParameters();
    void updateTranslationFromParameters();
    Eigen::Matrix2d computeAiMatrix(const Eigen::Vector2d &p, const Eigen::Vector2d &q) const;

    template <typename S1>
    Eigen::Matrix3d covariance(Retriever<S1> &retriever) ;
    template <typename S1>
    Eigen::Matrix3d covariance(InformedRetriever<S1> &retriever);
    template <typename S1>
    Eigen::Matrix3d covariance(FullyInformedRetriever<S1> &retriever);

private:
    Retriever<S> *retriever;
    QList<int> *associations;
    double atheta, btheta;          // theta = atheta^-1 * btheta
    Eigen::Vector2d bt;             // t = At^-1 * bt
    Eigen::Matrix2d At;
    double angle;
    Eigen::Vector2d translation, puretranslation;
};

template <typename S, typename W>
void DecoupledEstimator<S, W>::setRetriever(Retriever<S> &retr)
{
    this->retriever = &retr;
}

template <typename S, typename W>
void DecoupledEstimator<S, W>::setAssociations(QList<int> associations[])
{
    this->associations = associations;
}

template <typename S, typename W>
Eigen::Vector3d DecoupledEstimator<S, W>::rototranslation() const
{
    return Eigen::Vector3d(translation[0], translation[1], angle);
}

template <typename S, typename W>
Eigen::Vector3d DecoupledEstimator<S, W>::pureRototranslation() const
{
    return Eigen::Vector3d(puretranslation[0], puretranslation[1], angle);
}

template <typename S, typename W>
void DecoupledEstimator<S, W>::updateRotationFromParameters()
{
    angle = btheta;

    /* Avoid NaN values, should this ever happen */
    if(!almostEqual(atheta, 0, 1e-6))
        angle /= atheta;
}

template <typename S, typename W>
void DecoupledEstimator<S, W>::updateTranslationFromParameters()
{
#ifdef SM_USE_STANDARD_INVERSE
    /* Solution with standard inverse of A */
    puretranslation = - At.ldlt().solve(bt);
#else
    /* Solution with pseudo-inverse of A */
    puretranslation = - Support::pseudoInverse(At, 0.05) * bt;
#endif
}

template <typename S, typename W>
Eigen::Matrix2d DecoupledEstimator<S, W>::computeAiMatrix(
        const Eigen::Vector2d &p, const Eigen::Vector2d &q) const
{
    const double px_qx = p.x() - q.x(), py_qy = p.y() - q.y();
    Eigen::Matrix2d Ai;
    Ai <<
             py_qy * py_qy, -px_qx * py_qy,
            -px_qx * py_qy,  px_qx * px_qx;

    return Ai / (px_qx * px_qx + py_qy * py_qy);
}

template <typename S, typename W>
void DecoupledEstimator<S, W>::estimate()
{
    const int queryCount = retriever->querySegmentCount();

    /* Find optimal angle */
    atheta = btheta = 0;
    for(int i = 0; i < queryCount; i++) {
        const double segAngle = retriever->querySegment(i).angle();

        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            const double weight = W::angleWeight(retriever->derived(), i, *it);

            // TODO: il linewrap non mi convince, è sufficiente?
            btheta += weight * linewrap(retriever->mapSegment(*it).angle() - segAngle);
            atheta += weight;
        }

    }

    updateRotationFromParameters();

    /* Find optimal translation */
    Eigen::Vector2d t;
    Geometry::Rototranslation rt(retriever->queryPose().translation(), angle);
    At = Eigen::Matrix2d::Zero();
    bt = Eigen::Vector2d::Zero();

    for(int i = 0; i < queryCount; i++) {
        const Geometry::LineSegment &query = retriever->querySegment(i);
        const Eigen::Vector2d p = rt * query.p1();
        const Eigen::Matrix2d Ai = computeAiMatrix(p, rt * query.p2());
        double sumofweights = 0;

        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            const Eigen::Vector2d c = retriever->mapSegment(*it).centroid();
            double weight = W::translationWeight(retriever->derived(), i, *it);
            bt += weight * Ai * (p - c);
            sumofweights += weight;
        }

        At += sumofweights * Ai;
    }

    updateTranslationFromParameters();
    translation = puretranslation + rt.translation();
}

template <typename S, typename W>
bool DecoupledEstimator<S, W>::isCompatible(
        const Eigen::Vector3d &rt, double anglethresh, double traslthresh)
{
    if(std::abs(rt[2] - angle) > anglethresh) {
        return false;
    } else {
        const double querynorm = rt.block<2,1>(0,0).norm();
        const double normproj = puretranslation.dot(rt.block<2,1>(0,0)) / querynorm;
        if(std::abs(normproj - querynorm) > traslthresh) {
            return false;
        } else {
            return true;
        }
    }
}

template <typename S, typename W>
Eigen::Vector3d DecoupledEstimator<S, W>::singleAssociationEstimate(int scanidx, int refidx) const
{
    const Geometry::LineSegment &query = retriever->querySegment(scanidx);
    const Geometry::LineSegment &refseg = retriever->mapSegment(refidx);
    double angle = linewrap(refseg.angle() - query.angle());
    const Geometry::Rototranslation rt(retriever->queryPose().translation(), angle);
    const Eigen::Vector2d p = rt * query.p1(), c = refseg.centroid();
    const Eigen::Matrix2d Ai = computeAiMatrix(p, rt * query.p2());
    const Eigen::Vector2d translation = - Ai * (p - c);
    return Eigen::Vector3d(translation[0], translation[1], angle);
}

template <typename S, typename W>
Eigen::Vector2d DecoupledEstimator<S, W>::singleAssociationDeviation(int scanidx, int refidx) const
{
    return deviation(
            singleAssociationEstimate(scanidx, refidx),
            Eigen::Vector3d(puretranslation[0], puretranslation[1], angle));
}

template <typename S, typename W>
inline Eigen::Vector2d DecoupledEstimator<S, W>::deviation(
        const Eigen::Vector3d &query, const Eigen::Vector3d &ref)
{
/*
    const double refnorm = ref.norm();
    const double normproj = query.block<2,1>(0,0).squaredNorm() * refnorm /
                ref.block<2,1>(0,0).dot(query.block<2,1>(0,0));
    return Eigen::Vector2d(std::abs(query[2] - ref[2]), std::abs(normproj - refnorm));
*/
    const double querynorm = query.block<2,1>(0,0).norm();
    const double normproj = ref.block<2,1>(0,0).dot(query.block<2,1>(0,0)) / querynorm;
    return Eigen::Vector2d(std::abs(query[2] - ref[2]), std::abs(normproj - querynorm));
}



template <typename S, typename W> template <typename S1>
inline Eigen::Matrix3d DecoupledEstimator<S, W>::covariance(Retriever<S1> &retriever)
{
    Q_UNUSED(retriever)
    return Eigen::Matrix3d::Identity();
}

template <typename S, typename W> template <typename S1>
inline Eigen::Matrix3d DecoupledEstimator<S, W>::covariance(InformedRetriever<S1> &retriever)
{
    Q_UNUSED(retriever)
    return Eigen::Matrix3d::Identity();
}

template <typename S, typename W> template <typename S1>
inline Eigen::Matrix3d DecoupledEstimator<S, W>::covariance(FullyInformedRetriever<S1> &retriever)
{
    static const double stdDevLimit = 40;
    const int queryCount = retriever.querySegmentCount();
    const Geometry::Rototranslation rt(retriever.queryPose().translation(), angle);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

    for(int i = 0; i < queryCount; i++) {
        /* Assumes all segments are uncorrelated */
        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            const Geometry::UncertainLineSegment query = rt *
                    Geometry::UncertainLineSegment(retriever.querySegment(i),
                            retriever.querySegmentScanCovariance(i));
            const Geometry::LineSegment &map = retriever.mapSegment(*it);
            const double wa = W::angleWeight(retriever.derived(), i, *it);
            const double wt = W::translationWeight(retriever.derived(), i, *it);
            Eigen::Matrix<double, 3, 4> Jq, Jr;

            const double alpha1 = query.alpha();
            const double rho1   = query.rho();
            const double alpha2 = map.alpha();
            const double rho2   = map.rho();
            const double tx     = translation.x();
            const double ty     = translation.y();

            const double tmp0 = 2*alpha1;
            const double tmp1 = std::cos(tmp0);
            const double tmp2 = map.psi1() + map.psi2();
            const double tmp3 = -alpha2 + tmp0;
            const double tmp4 = std::cos(tmp3);
            const double tmp5 = std::sin(alpha1);
            const double tmp6 = std::sin(tmp0);
            const double tmp7 = std::sin(tmp3);
            const double tmp8 = std::cos(alpha1);
            const double tmp9 = alpha1 - alpha2;
            const double tmp10 = std::cos(tmp9);
            const double tmp11 = std::sin(tmp9);
            const double tmp12 = -2*rho2*tmp11 + tmp10*tmp2;
            const double tmp13 = -(tmp11*tmp8*wt)/2;
            const double tmp14 = -(tmp11*tmp5*wt)/2;

            Jq <<
                    -((tmp2*tmp4 + 2*rho1*tmp5 - 2*rho2*tmp7 + 4*tmp1*tx + 4*tmp6*ty)*wt)/2, tmp8*wt, 0, 0,
                    -(tmp2*tmp7*wt)/2 + (-(rho2*tmp4) + rho1*tmp8 - 2*tmp6*tx + 2*tmp1*ty)*wt, tmp5*wt, 0, 0,
                    wa, 0, 0, 0;

            Jr <<
                    (tmp12*tmp8*wt)/2, -(tmp10*tmp8*wt), tmp13, tmp13,
                    (tmp12*tmp5*wt)/2, -(tmp10*tmp5*wt), tmp14, tmp14,
                    -wa, 0, 0, 0;

            cov +=
                    Jq * query.covariance() * Jq.transpose() +
                    Jr * retriever.mapSegmentScanCovariance(*it) * Jr.transpose();
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0),
            Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d A;
    A <<
            -Support::cappedInverse(At, stdDevLimit, 0.05), Eigen::Vector2d::Zero(),
            Eigen::RowVector2d::Zero(), -1/atheta;

    if(almostEqual(svd.singularValues().y(), 0, 0.05)) {
        Eigen::Vector2d singularValues = svd.singularValues();
        singularValues[1] = 1 / stdDevLimit;
        cov.block<2,2>(0,0) = svd.matrixU() * singularValues.asDiagonal() * svd.matrixV().transpose();
    }

    return A * cov * A.transpose();
}

template <typename S, typename W>
inline Eigen::Matrix3d DecoupledEstimator<S, W>::covariance()
{
    return covariance(this->retriever->derived());
}

template <typename S, typename W>
void DecoupledEstimator<S, W>::removeAssociation(int scanidx, int refidx)
{
#if 0
    const double oldangle = angle;
    const Geometry::UncertainLineSegment &scanseg = s->getSegments()[scanidx];
    const T &refseg = *(*ref)[refidx];
    const double wa = angleWeight(scanseg, refseg);

    btheta -= wa * linewrap(refseg.angle() - scanseg.angle());
    atheta -= wa;

    updateRotationFromParameters();

    const Geometry::Rototranslation rot(oldangle);
    const Eigen::Vector2d p = rot * scanseg.p1(), c = refseg.centroid();
    const Eigen::Matrix2d Ai = computeAiMatrix(p, rot * scanseg.p2());
    const double wt = translationWeight(scanseg, refseg);
    At -= Ai * wt;
    bt -= Ai * (p - c) * wt;
    //ldbg << "wt*bti=" << Ai * (p - c) * wt << endl;

    const double ct = std::cos(angle - oldangle), st = std::sin(angle - oldangle);
    Eigen::Matrix2d H;
    H <<
          ct, -st,
          st,  ct;

    At = (H * At  * H.transpose()).eval();
    bt = (H * bt).eval();

    updateTranslationFromParameters();

    associations[scanidx].removeOne(refidx);
#else
    associations[scanidx].removeOne(refidx);
    estimate();
#endif
}



} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* DECOUPLEDESTIMATOR_H_ */
