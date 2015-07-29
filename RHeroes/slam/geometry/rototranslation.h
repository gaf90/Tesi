/*
 * rototranslation.h
 *
 *  Created on: 06/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef ROTOTRANSLATION_H_
#define ROTOTRANSLATION_H_

#include "data/pose.h"
#include "point.h"
#include "linesegment.h"
#include "slam/utilities.h"
#include <cmath>
#include <Eigen/Core>

namespace SLAM {
namespace Geometry {

class Rototranslation
{
public:
    Rototranslation(double angle = 0);
    Rototranslation(double tranx, double trany, double angle = 0);
    /**
     * Defines a rotation around the point "center" by an amount "angle".
     *
     * NOTE: this is different than defining a rototranslation with angular displacement "angle"
     * and translation equal to the coordinates of the point "center".
     *
     * @param center Center point around which the rotation should occur
     * @param angle Rotation amount, in radians
     */
    Rototranslation(const Point &center, double angle = 0);
    Rototranslation(const Data::Pose &pose);
    Rototranslation(const Eigen::Vector3d &vec);
    Rototranslation(const Data::Pose &from, const Data::Pose &to);
    ~Rototranslation();

    double angle() const;
    double cosAngle() const;
    double sinAngle() const;
    double tx() const;
    double ty() const;
    Eigen::Vector2d translation() const;

    Rototranslation inverse() const;
    Rototranslation operator*(const Rototranslation &rt) const;
    Point operator*(const Point &p) const;
    LineSegment operator*(const LineSegment &l) const;
    Eigen::Vector3d operator*(const Eigen::Vector3d &v) const;
    Data::Pose operator*(const Data::Pose &pose) const;
    //UncertainLineSegment operator*(const UncertainLineSegment &l) const;

    Rototranslation &operator=(const Rototranslation &rt);
    Eigen::Matrix3d matrixForm() const;
    Eigen::Vector3d vectorForm() const;
    operator Eigen::Vector3d() const;

protected:
    Rototranslation(double m11, double m13, double m21, double m23);

    double m11, m13;
    double m21, m23;
};

inline Rototranslation::Rototranslation(double angle) :
    m11(std::cos(angle)), m13(0), m21(std::sin(angle)), m23(0)
{
}

inline Rototranslation::Rototranslation(double tranx, double trany, double angle) :
    m11(std::cos(angle)), m13(tranx), m21(std::sin(angle)), m23(trany)
{
}

inline Rototranslation::Rototranslation(const Point &center, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);

    m11 =  c; m13 = (1 - c) * center.x() + s * center.y();
    m21 =  s; m23 = (1 - c) * center.y() - s * center.x();
}


inline Rototranslation::Rototranslation(const Data::Pose &pose) :
    m11(std::cos(pose.theta())), m13(pose.x()), m21(std::sin(pose.theta())), m23(pose.y())
{
}


inline Rototranslation::Rototranslation(const Eigen::Vector3d &vec) :
    m11(std::cos((double) vec[2])), m13(vec[0]), m21(std::sin((double) vec[2])), m23(vec[1])
{
}

inline Rototranslation::Rototranslation(const Data::Pose &from, const Data::Pose &to)
{
    (*this) = Rototranslation(from).inverse() * Rototranslation(to);
}

inline Rototranslation::Rototranslation(double m11, double m13, double m21, double m23) :
    m11(m11), m13(m13), m21(m21), m23(m23)
{
}

inline Rototranslation::~Rototranslation()
{
}

inline double Rototranslation::angle() const
{
    return std::atan2(m21, m11);
}

inline double Rototranslation::cosAngle() const
{
    return m11;
}

inline double Rototranslation::sinAngle() const
{
    return m21;
}

inline double Rototranslation::tx() const
{
    return m13;
}

inline double Rototranslation::ty() const
{
    return m23;
}

inline Eigen::Vector2d Rototranslation::translation() const
{
    return Eigen::Vector2d(m13, m23);
}

inline Rototranslation Rototranslation::inverse() const
{
    return Rototranslation(
                 m11, - m13 * m11 - m23 * m21,
                -m21, - m23 * m11 + m13 * m21);
}

inline Rototranslation Rototranslation::operator*(const Rototranslation &rt) const
{
    return Rototranslation(
                m11 * rt.m11 - m21 * rt.m21,
                    m13 + m11 * rt.m13 - m21 * rt.m23,
                m21 * rt.m11 + m11 * rt.m21,
                    m23 + m21 * rt.m13 + m11 * rt.m23);
}


inline Point Rototranslation::operator*(const Point &p) const
{
    return Point(m11 * p.x() - m21 * p.y() + m13, m21 * p.x() + m11 * p.y() + m23);
}

inline LineSegment Rototranslation::operator*(const LineSegment &l) const
{
    return LineSegment((*this) * l.p1(), (*this) * l.p2());
}

inline Eigen::Vector3d Rototranslation::operator*(const Eigen::Vector3d &v) const
{
    return Eigen::Vector3d(v[0] * m11 - v[1] * m21 + m13, v[0] * m21 + v[1] * m11 + m23,
            wrap(v[2] + angle()));
}

inline Data::Pose Rototranslation::operator*(const Data::Pose &p) const
{
    return Eigen::Vector3d(p.x() * m11 - p.y() * m21 + m13, p.x() * m21 + p.y() * m11 + m23,
            wrap(p.theta() + angle()));
}

inline Rototranslation &Rototranslation::operator=(const Rototranslation &rt)
{
    m11 = rt.m11; m13 = rt.m13;
    m21 = rt.m21; m23 = rt.m23;
    return *this;
}

inline Eigen::Matrix3d Rototranslation::matrixForm() const
{
    Eigen::Matrix3d m;
    m <<
            m11, -m21, m13,
            m21,  m11, m23,
              0,    0,   1;
    return m;
}

inline Eigen::Vector3d Rototranslation::vectorForm() const
{
    return *this;
}

inline Rototranslation::operator Eigen::Vector3d() const
{
    return Eigen::Vector3d(m13, m23, angle());
}

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* ROTOTRANSLATION_H_ */
