/*
 * pseudoinverse.h
 *
 *  Created on: 08/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef PSEUDOINVERSE_H_
#define PSEUDOINVERSE_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include "shared/utilities.h"

namespace SLAM {
namespace Support {

template <int N>
inline Eigen::Matrix<double, N, N> pseudoInverse(
        const Eigen::Matrix<double, N, N> &m, double threshold = 1e-6)
{
    Eigen::JacobiSVD<Eigen::Matrix<double, N, N> > svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, N, 1> sigma = svd.singularValues();
    for(int i = 0; i < N; i++) {
        sigma[i] = almostEqual(sigma[i], 0, threshold) ? 0 : 1 / sigma[i];
    }

    return svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();
}

template <int N>
inline Eigen::Matrix<double, N, N> cappedInverse(
        const Eigen::Matrix<double, N, N> &m, double cap, double threshold = 1e-6)
{
    Eigen::JacobiSVD<Eigen::Matrix<double, N, N> > svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, N, 1> sigma = svd.singularValues();
    for(int i = 0; i < N; i++) {
        /*
            Ignores the sign of sigma[i], which is good when we know that all singular values
            should be positive (positive defined matrices), not so good in the general case.
            Fortunately the general case is not necessary for the SLAM.
        */
        sigma[i] = almostEqual(sigma[i], 0, threshold) ? cap : 1 / sigma[i];
    }

    return svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();
}

} /* namespace Support */
} /* namespace SLAM */

#endif /* PSEUDOINVERSE_H_ */
