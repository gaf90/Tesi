/*
 * nearestmatrices.h
 *
 *  Created on: 01/feb/2013
 *      Author: Mladen Mazuran
 */

#ifndef NEARESTMATRICES_H_
#define NEARESTMATRICES_H_

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include "shared/utilities.h"

namespace SLAM {
namespace Support {

/*
    Fan, K. and Hoffman, A.J. (1955) Some metric inequalities in the space of matrices,
    Proc. Amer. Math. Soc. 6, 111–116.
*/
template <int N>
inline Eigen::Matrix<double, N, N> nearestSymmetric(const Eigen::Matrix<double, N, N> &m)
{
    return (m + m.transpose()) / 2;
}


/*
    Higham, N.J. (1988a) Computing a nearest symmetric positive semidefinite matrix,
    Linear Algebra and Appl. 103, 103–118.
*/
template <int N>
inline Eigen::Matrix<double, N, N> nearestDefinitePositive(
        const Eigen::Matrix<double, N, N> &m, double diagonalDelta = 1e-6)
{
    if(!m.ldlt().isPositive()) {
        Eigen::Matrix<double, N, N> s = nearestSymmetric(m);
        Eigen::JacobiSVD<Eigen::Matrix<double, N, N> > svd(s, Eigen::ComputeFullV);
        return (svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose() +
                s) / 2 + diagonalDelta * Eigen::Matrix<double, N, N>::Identity();
    } else {
        return m;
    }
}


} /* namespace Support */
} /* namespace SLAM */

#endif /* NEARESTMATRICES_H_ */
