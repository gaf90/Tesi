/*
 * ligriffithsicl.h
 *
 *  Created on: 27/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef LIGRIFFITHSICL_H_
#define LIGRIFFITHSICL_H_

#include "slam/constants.h"
#include "slam/utilities.h"
#include "associationligriffiths.h"
#include "scanmatcher.h"
#include <Eigen/SVD>
#include <Eigen/LU>
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
#   include "slam/dataset/mathematicavisualization.h"
#endif

namespace SLAM {
namespace ScanMatching {

/*
 * Li, Q., Griffiths, J.: Iterative closest geometric objects registration.
 * Computers & Mathematics with Applications 40(10-11) (2000) 1171–1188
 */
template <typename T>
class LiGriffithsICL : public ScanMatcher<T> {
public:
	bool run();
    Eigen::Matrix3d covariance();

private:
	Eigen::Vector3d bestRototranslation() const;
};

template <typename T>
Eigen::Matrix3d LiGriffithsICL<T>::covariance()
{
    return 1e-2 * Eigen::Matrix3d::Identity();
}

template <typename T>
bool LiGriffithsICL<T>::run()
{
    int counter = 0;
    Eigen::Vector3d stepRT;

    do {
        bool foundSomething = AssociationLiGriffiths::lookup<1>(*this->retriever, this->assoc);

        if(!foundSomething) {
            this->rt = this->retriever->queryPose();
            return false;
        }

#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
        Dataset::MathematicaVisualization::associationMap(*this->retriever, this->assoc);
#endif

        stepRT = bestRototranslation();

        this->retriever->applyTransformation(Geometry::Rototranslation(stepRT));
        counter++;
    } while((SQUARE(stepRT[0]) + SQUARE(stepRT[1]) > SQUARE(SM_ICL_CONVERGENCE_THRESHOLD) ||
                std::abs(wrap(stepRT[2])) > SM_ICL_CONVERGENCE_THRESHOLD) &&
                counter < SM_ICL_MAXIMUM_ITERATIONS);

    this->rt = this->retriever->queryPose();
	return true;
}

template <typename T>
Eigen::Vector3d LiGriffithsICL<T>::bestRototranslation() const
{
    const int querySegmentCount = this->retriever->querySegmentCount();
    Eigen::Vector2d pbar = Eigen::Vector2d::Zero();
    Eigen::Vector2d qbar = Eigen::Vector2d::Zero();
    Eigen::Matrix2d A    = Eigen::Matrix2d::Zero();
    double w = 0;

    for(int i = 0; i < querySegmentCount; i++) {
        const Geometry::LineSegment &p = this->retriever->querySegment(i);
        for(QList<int>::const_iterator it = this->assoc[i].begin(),
                end = this->assoc[i].end(); it != end; ++it) {
            const Geometry::LineSegment &q = this->retriever->mapSegment(*it);
            const double wn = p.length() + q.length();
            pbar += wn * p.centroid().vector();
            qbar += wn * q.centroid().vector();
            w += wn;
        }
    }

    pbar /= w;
    qbar /= w;

    for(int i = 0; i < querySegmentCount; i++) {
        const Geometry::LineSegment &p = this->retriever->querySegment(i);
        for(QList<int>::const_iterator it = this->assoc[i].begin(),
                end = this->assoc[i].end(); it != end; ++it) {
            const Geometry::LineSegment &q = this->retriever->mapSegment(*it);

            const Eigen::Vector2d p1bar = p.p1().vector() - pbar, p2bar = p.p2().vector() - pbar;
            const Eigen::Vector2d q1bar = q.p1().vector() - qbar, q2bar = q.p2().vector() - qbar;

            A += (2 * q1bar * p1bar.transpose() + 2 * q2bar * p2bar.transpose() +
                    q1bar * p2bar.transpose() + q2bar * p1bar.transpose()) *
                            (p.length() + q.length()) / 6;
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
    if(R.determinant() < 0) R.block<2, 1>(0, 1) *= -1;

    Eigen::Vector2d t = qbar - R * pbar;
    return Eigen::Vector3d(t[0], t[1], std::atan2(R(1, 0), R(0, 0)));
}

} /* namespace ScanMatching */
} /* namespace SLAM */
#endif /* LIGRIFFITHSICL_H_ */
