/*
 * point.h
 *
 *  Created on: 05/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef POINT_H_
#define POINT_H_

#include "data/serializable.h"
#include "shared/logger.h"
#include <cmath>
#include <QPointF>

namespace SLAM {
namespace Geometry {

class Point : public Data::Serializable
{
public:
    Point();
    Point(double x, double y);
    Point(const Eigen::Vector2d &p);
    Point(const Point &p);

    double angle() const;

    double norm2() const;
    double norm() const;

    double distance2(const Point &p) const;
    double distance(const Point &p) const;

    double x() const;
    double y() const;
    void setX(double x);
    void setY(double y);

    double &rx();
    double &ry();

    Eigen::Vector2d vector() const;
    operator Eigen::Vector2d() const;
    operator QPointF() const;

    bool almostEqual(const Point &p, double threshold = 1e-6) const;

    bool operator==(const Point &p) const;
    bool operator!=(const Point &p) const;
    Point operator-() const;
    Point operator-(const Point &p) const;
    Point operator+(const Point &p) const;
    Point operator/(double a) const;
    Point operator*(double a) const;

    Point &operator+=(const Point &p);
    Point &operator-=(const Point &p);
    Point &operator*=(double a);
    Point &operator/=(double a);

    /**
     * @returns Scalar product of @c this with @c p
     */
    double operator*(const Point &p) const;

    friend Point operator*(double a, const Point &p);

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    friend LoggerStream &operator<<(LoggerStream &stream, const Point &point);
    friend LoggerStream &operator<<(LoggerStream &stream, const Point *point);

private:
    double xp;
    double yp;
};

inline Point::Point(): xp(0), yp(0)
{
}

inline Point::Point(double x, double y) : xp(x), yp(y)
{
}

inline Point::Point(const Eigen::Vector2d &p) : xp(p.x()), yp(p.y())
{
}

inline Point::Point(const Point &p) : Data::Serializable(), xp(p.xp), yp(p.yp)
{
}

inline double Point::angle() const
{
    return std::atan2(yp, xp);
}

inline double Point::norm2() const
{
    return xp * xp + yp * yp;
}

inline double Point::norm() const
{
    return std::sqrt(norm2());
}

inline double Point::distance2(const Point &p) const
{
    return (xp - p.xp) * (xp - p.xp) + (yp - p.yp) * (yp - p.yp);
}

inline double Point::distance(const Point &p) const
{
    return std::sqrt(distance2(p));
}

inline double Point::x() const
{
    return xp;
}

inline double Point::y() const
{
    return yp;
}

inline void Point::setX(double x)
{
    xp = x;
}

inline void Point::setY(double y)
{
    yp = y;
}

inline double &Point::rx()
{
    return xp;
}

inline double &Point::ry()
{
    return yp;
}


inline Eigen::Vector2d Point::vector() const
{
    return Eigen::Vector2d(xp, yp);
}

inline Point::operator Eigen::Vector2d() const
{
    return vector();
}

inline Point::operator QPointF() const
{
    return QPointF(xp, yp);
}

inline bool Point::almostEqual(const Point &p, double threshold) const
{
    return distance2(p) < threshold * threshold;
}

inline bool Point::operator==(const Point &p) const
{
    return xp == p.xp && yp == p.yp;
}

inline bool Point::operator!=(const Point &p) const
{
    return xp != p.xp || yp != p.yp;
}

inline Point Point::operator-() const
{
    return Point(-xp, -yp);
}

inline Point Point::operator-(const Point &p) const
{
    return Point(xp - p.xp, yp - p.yp);
}

inline Point Point::operator+(const Point &p) const
{
    return Point(xp + p.xp, yp + p.yp);
}

inline Point Point::operator/(double a) const
{
    return Point(xp / a, yp / a);
}

inline Point Point::operator*(double a) const
{
    return Point(a * xp, a * yp);
}

inline double Point::operator*(const Point &p) const
{
    return xp * p.xp + yp * p.yp;
}

inline Point operator*(double a, const Point &p)
{
    return Point(a * p.xp, a * p.yp);
}

inline Point &Point::operator-=(const Point &p)
{
    xp -= p.xp; yp -= p.yp;
    return *this;
}

inline Point &Point::operator+=(const Point &p)
{
    xp += p.xp; yp += p.yp;
    return *this;
}

inline Point &Point::operator/=(double a)
{
    xp /= a; yp /= a;
    return *this;
}

inline Point &Point::operator*=(double a)
{
    xp *= a; yp *= a;
    return *this;
}


} /* namespace Geometry */
} /* namespace SLAM */

#endif /* POINT_H_ */
