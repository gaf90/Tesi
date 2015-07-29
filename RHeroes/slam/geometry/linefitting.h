/*
 * linefitting.h
 *
 *  Created on: 23/mag/2012
 *      Author: Mladen Mazuran
 */

#ifndef LINEFITTING_H_
#define LINEFITTING_H_

#include "slam/utilities.h"
#include "slam/support/indexset.h"
#include "shared/config.h"
#include "pointscan.h"
#include <cmath>
#include <algorithm>
#include <Eigen/SVD>
#include <Eigen/LU>

namespace SLAM {
namespace Geometry {

namespace __internal {

inline Eigen::Matrix4d mlLineCovariance(
        double a, double r, const PointScan &scan, const Support::IndexSet &idx)
{
    Q_UNUSED(r)

    Eigen::Matrix2d info = Eigen::Matrix2d::Zero();

    fforeach(int i, idx) {
        const double range = scan.range(i), angle = scan.angle(i);
        const double s = std::sin(a - angle), c = std::cos(a - angle);

        const double sigma2 =
                Config::SLAM::laserRangeVariance * square(c) +
                Config::SLAM::laserAngleVariance * square(range) * square(s);

        Eigen::Matrix2d JtJ;
        JtJ <<
                square(range) * square(s), range * s,
                range * s, 1;

        info += JtJ / sigma2;
    }

    const double r1 = scan.range(idx.indexBegin()),   t1 = scan.angle(idx.indexBegin());
    const double r2 = scan.range(idx.indexEnd() - 1), t2 = scan.angle(idx.indexEnd() - 1);
    const double p11 = square(r1) * Config::SLAM::laserAngleVariance * square(std::cos(a - t1)) +
            Config::SLAM::laserRangeVariance * square(std::sin(a - t1));
    const double p22 = square(r2) * Config::SLAM::laserAngleVariance * square(std::cos(a - t2)) +
            Config::SLAM::laserRangeVariance * square(std::sin(a - t2));

    const Eigen::Matrix2d linecov = info.inverse();
    const Eigen::Matrix2d endcov = Eigen::Vector2d(p11, p22).asDiagonal();
    Eigen::Matrix4d cov;
    cov <<
            linecov, Eigen::Matrix2d::Zero(),
            Eigen::Matrix2d::Zero(), endcov;
    return cov;
}


inline Eigen::Matrix4d weightedLineCovariance(
        double a, double r, const PointScan &scan, const Support::IndexSet &idx)
{
    double sum1 = 0, sum2 = 0, prr1 = 0, gt = 0;

    fforeach(int i, idx) {
        const double range = scan.range(i), angle = scan.angle(i);
        const double sk = std::sin(a - angle), ck = std::cos(a - angle);

        const double tmp0 = range * ck - r;
        const double tmp1 = range * range;
        const double tmp2 = sk * sk;
        const double tmp3 = ck * ck;
        const double tmp4 = 2 * (tmp1 * Config::SLAM::laserAngleVariance -
                Config::SLAM::laserRangeVariance);

        const double a1k = square(tmp0);
        const double a2k = -2 * range * sk * tmp0;
        const double a3k = 2 * (tmp1 * tmp2 - range * ck * tmp0);

        const double b1k = Config::SLAM::laserRangeVariance * tmp3 +
                Config::SLAM::laserAngleVariance * tmp1 * tmp2;
        const double b2k = tmp4 * ck * sk;
        const double b3k = tmp4 * (tmp3 - tmp2);

        prr1 += 1 / b1k;
        sum1 += 2 * range * sk / b1k;
        sum2 += 4 * tmp1 * tmp2 / b1k;
        gt += ((a3k * b1k - a1k * b3k) * b1k - 2 * (a2k * b1k - a1k * b2k) * b2k) /
                (b1k * b1k * b1k);
    }

    const double r1 = scan.range(idx.indexBegin()),   t1 = scan.angle(idx.indexBegin());
    const double r2 = scan.range(idx.indexEnd() - 1), t2 = scan.angle(idx.indexEnd() - 1);

    const double prr = 1 / prr1;
    const double pra = - prr * sum1 / gt;
    const double paa = sum2 / square(gt);
    const double p11 = square(r1) * Config::SLAM::laserAngleVariance * square(std::cos(a - t1)) +
            Config::SLAM::laserRangeVariance * square(std::sin(a - t1));
    const double p22 = square(r2) * Config::SLAM::laserAngleVariance * square(std::cos(a - t2)) +
            Config::SLAM::laserRangeVariance * square(std::sin(a - t2));

    Eigen::Matrix4d cov;
    cov <<
            paa, pra,   0,   0,
            pra, prr,   0,   0,
              0,   0, p11,   0,
              0,   0,   0, p22;
    return cov;
}

inline double weightedRangeEstimate(double a, const PointScan &scan, const Support::IndexSet &idx)
{
    double sum1 = 0, sum2 = 0;
    fforeach(int i, idx) {
        const double range = scan.range(i), angle = scan.angle(i);
        const double sk = std::sin(a - angle), ck = std::cos(a - angle);
        const double pk = Config::SLAM::laserRangeVariance * ck * ck +
                Config::SLAM::laserAngleVariance * range * range * sk * sk;
        sum1 += range * ck / pk;
        sum2 += 1 / pk;
    }
    return sum1 / sum2;
}

inline double weightedAngleEstimate(
        double a, double r, const PointScan &scan, const Support::IndexSet &idx)
{
    double sum1 = 0, sum2 = 0;
    fforeach(int i, idx) {
        const double range = scan.range(i), angle = scan.angle(i);
        const double sk = std::sin(a - angle), ck = std::cos(a - angle);

        const double tmp0 = range * ck - r;
        const double tmp1 = range * range;
        const double tmp2 = sk * sk;
        const double tmp3 = ck * ck;
        const double tmp4 = 2 * (tmp1 * Config::SLAM::laserAngleVariance -
                Config::SLAM::laserRangeVariance);

        const double a1k = square(tmp0);
        const double a2k = -2 * range * sk * tmp0;
        const double a3k = 2 * (tmp1 * tmp2 - range * ck * tmp0);

        const double b1k = Config::SLAM::laserRangeVariance * tmp3 + Config::SLAM::laserAngleVariance * tmp1 * tmp2;
        const double b2k = tmp4 * ck * sk;
        const double b3k = tmp4 * (tmp3 - tmp2);

        const double tmp5 = b1k * b1k;

        sum1 += (b1k * a2k - a1k * b2k) / tmp5;
        sum2 += ((a3k * b1k - a1k * b3k) * b1k - 2 * (a2k * b1k - a1k * b2k) * b2k) /
                (tmp5 * b1k);
    }
    return a - sum1 / sum2;
}

} /* namespace __internal */

inline Geometry::UncertainLineSegment weightedLineFit(
        const PointScan &scan, const Support::IndexSet &idx)
{
    Geometry::LineSegment approx(scan[idx.indexBegin()], scan[idx.indexEnd() - 1]);
    /* Initial guess for the angle */
    double a = approx.angle() + M_PI_2;
    double r = 0;
    double preva = INFINITY, prevr = INFINITY;
    int i = 0;

    while(std::abs(preva - a) > 1e-3 && std::abs(prevr - r) > 1e-4) {
        preva = a; prevr = r;
        r = __internal::weightedRangeEstimate(a, scan, idx);
        a = __internal::weightedAngleEstimate(a, r, scan, idx);
        i++;
    }

    return Geometry::UncertainLineSegment(a, r, approx.p1(), approx.p2(),
            __internal::weightedLineCovariance(a, r, scan, idx));
}

inline const Geometry::UncertainLineSegment totalLeastSquaresLineFit(
        const PointScan &scan, const Support::IndexSet &idx)
{
    int n = idx.size();
    const Geometry::Point first = scan[idx.indexBegin()];
    const Geometry::Point last  = scan[idx.indexEnd() - 1];

    /* Calculate statistics on input points */
    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    Eigen::Matrix2d s = Eigen::Matrix2d::Zero();
    fforeach(int i, idx) {
        const Eigen::Vector2d x = scan[i];
        c += x;
        s += x * x.transpose();
    }

    Eigen::Matrix2d cov = (s - c * c.transpose() / n) / (n - 1);
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullV);
    Eigen::Vector2d v = svd.matrixV().col(0);

    /* We need the line in the "cos(alpha) x + sin(alpha) y = r" form */
    double alpha = std::atan2(v.x(), -v.y());
    /* Keep the same orientation */
    if(wrap(LineSegment(first, last).alpha() - alpha) > M_PI_2) {
        alpha = wrap(alpha - M_PI);
        c *= -1;
    }

    const double r = (c.y() * v.x() - c.x() * v.y()) / n;

    return Geometry::UncertainLineSegment(alpha, r, first, last,
            __internal::weightedLineCovariance(alpha, r, scan, idx));
}

namespace __internal {

inline const Geometry::LineSegment totalLeastTrimmedSquaresIteration(
        const Geometry::LineSegment &guess, const PointScan &scan, const Support::IndexSet &idx)
{
    static const int trimRatio = 0.4;
    const int n = idx.size() * (1 - trimRatio);

    QVector<std::pair<double, int> > costs(idx.size());
    int j = 0;
    fforeach(int i, idx) {
        costs[j++] = std::make_pair(guess.distance(scan[i]), i);
    }
    qSort(costs);

    /* Calculate statistics on input points */
    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    Eigen::Matrix2d s = Eigen::Matrix2d::Zero();
    for(int i = 0; i < n; i++) {
        const Eigen::Vector2d x = scan[costs[i].second];
        c += x;
        s += x * x.transpose();
    }

    Eigen::Matrix2d cov = (s - c * c.transpose() / n) / (n - 1);
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullV);
    Eigen::Vector2d v = svd.matrixV().col(0);
    c /= n;

    Geometry::LineSegment ls(c.x(), c.y(), v.x() + c.x(), v.y() + c.y());

    return Geometry::LineSegment(
            ls.project(scan[idx.indexBegin()]), ls.project(scan[idx.indexEnd() - 1]));
}

} /* namespace __internal */

inline const Geometry::UncertainLineSegment totalLeastTrimmedSquaresLineFit(
        const PointScan &scan, const Support::IndexSet &idx)
{
    int iter = 0;
    const Geometry::Point first = scan[idx.indexBegin()];
    const Geometry::Point last  = scan[idx.indexEnd() - 1];
    Geometry::LineSegment prevGuess, currentGuess(first, last);

    do {
        prevGuess = currentGuess;
        currentGuess = __internal::totalLeastTrimmedSquaresIteration(currentGuess, scan, idx);
    } while((prevGuess.lineVector() - currentGuess.lineVector()).norm() > 1e-4 && ++iter < 15);

    return Geometry::UncertainLineSegment(
            currentGuess, __internal::weightedLineCovariance(
                    currentGuess.alpha(), currentGuess.rho(), scan, idx));
}

/* Theil–Sen median estimator */
inline const Geometry::UncertainLineSegment medianLineFit(
        const PointScan &scan, const Support::IndexSet &idx)
{
    QList<double> angles;
    const Geometry::Point first = scan[idx.indexBegin()];
    const Geometry::Point last  = scan[idx.indexEnd() - 1];
    double offset = 0, angleGuess = Geometry::LineSegment(first, last).angle();
    /* Try to avoid discontinuity at -M_PI/M_PI by shifting */
    if(angleGuess > M_PI_2 || angleGuess < -M_PI_2)
        offset = M_PI;

    int count = 0;
    Geometry::Point centroid(0, 0);
    Support::IndexSet::const_iterator end = idx.end();
    for(Support::IndexSet::const_iterator it1 = idx.begin(); it1 != end; ++it1) {
        Support::IndexSet::const_iterator it2 = it1;
        const Geometry::Point &p1 = scan[*it1];
        for(++it2; it2 != end; ++it2) {
            const Geometry::Point &p2 = scan[*it2];
            angles.append(wrap(std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) - offset));
        }
        centroid += p1;
        count++;
    }
    centroid /= count;

    double medianAngle = wrap(medianInPlace(angles) + offset + M_PI_2);

    const double rho = centroid.y() * std::sin(medianAngle) + centroid.x() * std::cos(medianAngle);

    return Geometry::UncertainLineSegment(medianAngle, rho, first, last,
            __internal::weightedLineCovariance(medianAngle, rho, scan, idx));
}

/* Siegel's Theil–Sen median estimator variant */
inline const Geometry::UncertainLineSegment repeatedMedianLineFit(
        const PointScan &scan, const Support::IndexSet &idx)
{
    const int points = idx.size();
    QVector<double> angles(points - 1), angleMedians(points);
    const Geometry::Point first = scan[idx.indexBegin()];
    const Geometry::Point last  = scan[idx.indexEnd() - 1];
    double offset = 0, angleGuess = Geometry::LineSegment(first, last).angle();

    /* Try to avoid discontinuity at -M_PI/M_PI by shifting */
    if(angleGuess > M_PI_2 || angleGuess < -M_PI_2)
        offset = M_PI;

    int count = 0;
    Geometry::Point centroid(0, 0);
    Support::IndexSet::const_iterator end = idx.end();
    int i = 0;
    for(Support::IndexSet::const_iterator it1 = idx.begin(); it1 != end; ++it1, ++i) {
        Support::IndexSet::const_iterator it2 = it1;
        const Geometry::Point &p1 = scan[*it1];
        ++it2;
        for(int j = 0; it2 != end; ++it2, ++j) {
            const Geometry::Point &p2 = scan[*it2];
            angles[j] = wrap(std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) - offset);
        }
        centroid += p1;
        angleMedians[i] = medianInPlace(angles);
        count++;
    }
    centroid /= count;
    double medianAngle = wrap(medianInPlace(angleMedians) + offset + M_PI_2);

    const double rho =
            centroid.y() * std::sin(medianAngle) +
            centroid.x() * std::cos(medianAngle);

    return Geometry::UncertainLineSegment(medianAngle, rho, first, last,
                    __internal::weightedLineCovariance(medianAngle, rho, scan, idx));
}

} /* namespace Geometry */
} /* namespace SLAM */


#endif /* LINEFITTING_H_ */
