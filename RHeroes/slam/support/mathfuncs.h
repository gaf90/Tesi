/*
 * mathfuncs.h
 *
 *  Created on: 05/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef MATHFUNCS_H_
#define MATHFUNCS_H_

#include <qglobal.h>
#include <cmath>

namespace SLAM {

#ifndef isnan
#   if defined(Q_OS_WIN32)
#       define isnan _isnan
#   elif defined(Q_OS_MACX)
#       define isnan std::isnan
#   endif
#endif

template <typename T>
inline T square(T x)
{
    return x * x;
}

template <typename T>
inline T round(T x)
{
    return std::floor(x + T(0.5));
}

inline double wrap(double ang)
{
    while(ang > M_PI) ang -= 2 * M_PI;
    while(ang <= -M_PI) ang += 2 * M_PI;
    return ang;
}

inline double linewrap(double ang)
{
    ang = wrap(ang);
    if(ang > M_PI_2)
        return ang - M_PI;
    else if(ang <= -M_PI_2)
        return ang + M_PI;
    else
        return ang;
}

template <typename Container>
inline typename Container::value_type medianInPlace(Container &c) {
    const int middle = c.size() / 2;
    typename Container::iterator begin = c.begin(), end = c.end();

    std::nth_element(begin, begin + middle, end);

    if(c.size() % 2 == 0) {
        typename Container::value_type middlepart = *(begin + middle);
        std::nth_element(begin, begin + (middle - 1), end);
        return (middlepart + *(begin + (middle - 1))) / 2;
    } else {
        return *(begin + middle);
    }
}

template <typename Container>
inline typename Container::value_type median(const Container &c) {
    Container copy = c;
    return medianInPlace(copy);
}

inline double weightedLineAngle(double t1, double t2, double w1, double w2)
{
#if 0
    const double tdiff = t1 - t2;

    /* Determine weighted resulting angle */
    if(tdiff <= M_PI_2 && tdiff >= - M_PI_2) {
        return (w1 * t1 + w2 * t2) / (w1 + w2);
    } else if(tdiff <= M_PI && tdiff >= - M_PI) {
        return (w1 * t1 + w2 * wrap(t2 + M_PI)) / (w1 + w2);
    } else if(tdiff <= 3 * M_PI_2 && tdiff >= - 3 * M_PI_2) {
        return (w1 * t1 + w2 * wrap(t2 + M_PI)) / (w1 + w2) + M_PI;
    } else {
        return (w1 * wrap(t1 + M_PI) + w2 * wrap(t2 + M_PI)) / (w1 + w2) + M_PI;
    }
#else
    if(t1 < 0) t1 += M_PI;
    if(t2 < 0) t2 += M_PI;
    if(std::abs(t1 - t2) < M_PI_2) {
        return (w1 * t1 + w2 * t2) / (w1 + w2);
    } else if(t1 > M_PI_2) {
        return (w1 * (t1 - M_PI) + w2 * t2) / (w1 + w2);
    } else {
        return (w1 * t1 + w2 * (t2 - M_PI)) / (w1 + w2);
    }
#endif
}

#if 0
/* Assumes symmetric matrix */
#define BLOCK_INVERSE_2D(a, b, c) \
    if(!infinite[a]) { \
        double idet = 1 / (m(b,c)*m(b,c) - m(b,b)*m(c,c)); \
        ret(b,b) = - m(c,c) * idet; \
        ret(b,c) = m(b,c) * idet; \
        ret(c,b) = ret(b,c); \
        ret(c,c) = - m(b,b) * idet; \
    }

inline Eigen::Matrix3d safeInverse(const Eigen::Matrix3d &m)
{
    bool infinite[3] = { m(0,0) == INFINITY, m(1,1) == INFINITY, m(2,2) == INFINITY };
    int count = (infinite[0] ? 0 : 1) + (infinite[1] ? 0 : 1) + (infinite[2] ? 0 : 1);
    if(count == 3) {
        return m.inverse();
    } else if(count == 0) {
        return Eigen::Matrix3d::Zero();
    } else if(count == 1) {
        Eigen::Matrix3d ret = Eigen::Matrix3d::Zero();
        if(infinite[0]) ret(0,0) = 1 / m(0,0);
        if(infinite[1]) ret(1,1) = 1 / m(1,1);
        if(infinite[2]) ret(2,2) = 1 / m(2,2);
        return ret;
    } else {
        Eigen::Matrix3d ret = Eigen::Matrix3d::Zero();
        /* Assumed symmetric */
        BLOCK_INVERSE_2D(0, 1, 2)
        BLOCK_INVERSE_2D(1, 0, 2)
        BLOCK_INVERSE_2D(2, 0, 1)
        return ret;
    }
}

#undef BLOCK_INVERSE_2D
#endif

} /* namespace SLAM */

#endif /* MATHFUNCS_H_ */
