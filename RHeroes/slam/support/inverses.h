/*
 * inverses.h
 *
 *  Created on: 08/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef INVERSES_H_
#define INVERSES_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>
#include "shared/utilities.h"
#include "assert.h"

namespace SLAM {
namespace Support {

/* Lower MxM block of the inverse of m */
template <int M, int N>
inline Eigen::Matrix<double, M, M> lowerBlockInverse(const Eigen::Matrix<double, N, N> &m)
{
    static_assert(M < N);
    typedef const Eigen::Matrix<double, N, N> MatrixType;
    /*
         Let m = [ A B ; C D ], then what we seek is (D - C A^-1 B)^-1
    */
    const Eigen::Block<MatrixType, N - M, N - M> A = m.template block<N - M, N - M>(    0,     0);
    const Eigen::Block<MatrixType, N - M,     M> B = m.template block<N - M,     M>(    0, N - M);
    const Eigen::Block<MatrixType,     M, N - M> C = m.template block<    M, N - M>(N - M,     0);
    const Eigen::Block<MatrixType,     M,     M> D = m.template block<    M,     M>(N - M, N - M);

    return (D - C * A.inverse() * B).inverse();
}

/* Lower MxM block of the inverse of m */
template <int M>
inline Eigen::Matrix<double, M, M> lowerBlockInverse(const Eigen::MatrixXd &m)
{
    typedef const Eigen::MatrixXd MatrixType;

    const int N = m.cols();

    const Eigen::Block<MatrixType> A = m.block(    0,     0, N - M, N - M);
    const Eigen::Block<MatrixType> B = m.block(    0, N - M, N - M,     M);
    const Eigen::Block<MatrixType> C = m.block(N - M,     0,     M, N - M);
    const Eigen::Block<MatrixType> D = m.block(N - M, N - M,     M,     M);

    return (D - C * A.inverse() * B).inverse();
}

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

#endif /* INVERSES_H_ */
