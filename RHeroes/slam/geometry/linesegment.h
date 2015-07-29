/*
 * linesegment.h
 *
 *  Created on: 05/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include "slam/constants.h"
#include "shared/utilities.h"
#include "slam/utilities.h"
#include "point.h"
#include "data/serializable.h"
#include <QLineF>

namespace SLAM {
namespace Geometry {

class LineSegment : public Data::Serializable
{
public:
    enum VectorIndices {
        AlphaVectorIndex = 0,
        RhoVectorIndex = 1,
        Psi1VectorIndex = 2,
        Psi2VectorIndex = 3
    };

    /**
     * Initialises a nil line segment, from (0,0) to (0,0)
     */
    LineSegment();

    /**
     * Initialises a line segment with endpoints p1 and p2
     * @param p1 The first endpoint
     * @param p2 The second endpoint
     */
    LineSegment(const Point &p1, const Point &p2);

    /**
     * Initialises a line segment with endpoints (x1,y1) and (x2,y2)
     * @param x1 The x coordinate of the first endpoint
     * @param y1 The y coordinate of the first endpoint
     * @param x2 The x coordinate of the second endpoint
     * @param y2 The y coordinate of the second endpoint
     */
    LineSegment(double x1, double y1, double x2, double y2);

    /**
     * Copy constructor
     */
    LineSegment(const LineSegment &s);

    /**
     * @returns The first endpoint
     */
    const Point &p1() const;
    /**
     * @returns The second endpoint
     */
    const Point &p2() const;
    /**
     * Sets the first endpoint
     * @param p1 First endpoint
     */
    void setP1(const Point &p1);
    /**
     * Sets the second endpoint
     * @param p2 Second endpoint
     */
    void setP2(const Point &p2);

    /**
     * @param p Query point
     * @returns The squared point to line distance between the query point and the line identified
     *          by the segment
     */
    double distance2(const Point &p) const;   
    /**
     * @param p Query point
     * @returns The point to line distance between the query point and the line identified by the
     *          segment
     */
    double distance(const Point &p) const;


    /**
     * @returns The x coordinate of the first endpoint
     */
    double x1() const;
    /**
     * @returns The y coordinate of the first endpoint
     */
    double y1() const;

    /**
     * @returns The x coordinate of the second endpoint
     */
    double x2() const;
    /**
     * @returns The y coordinate of the second endpoint
     */
    double y2() const;

    /**
     * @returns The x variation between the two endpoints of the line segment
     */
    double dx() const;
    /**
     * @returns The y variation between the two endpoints of the line segment
     */
    double dy() const;

    /**
     * @returns The centroid of the line segment
     */
    Point centroid() const;
    /**
     * @returns The squared length of the line segment
     */
    double length2() const;
    /**
     * @returns The length of the line segment
     */
    double length() const;
    /**
     * @returns The angle of the segment (with respect to the x axis, counter-clockwise)
     */
    double angle() const;
    /**
     * @returns The angular coefficient of the line segment
     */
    double m() const;
    /**
     * @returns The intercept with y axis of the line identified by the segment
     */
    double q() const;

    double psi1() const;
    double psi2() const;
    double rho() const;
    double alpha() const;
    double psiProjection(const Point &p) const;

    /**
     * Projects p on the axis identified by the line segment
     *
     * @param p Point to project
     * @returns The projection point in x,y space
     */
    Point project(const Point &p) const;

    /**
     * Projects p on the axis identified by the line: a * x + b * y + c = 0
     *
     * @param a x coefficient
     * @param b y coefficient
     * @param c Additive coefficient
     * @param p Point to project
     * @returns The projection point in x,y space
     */
    static Point project(double a, double b, double c, const Point &p);

    /**
     * Projects point on the axis identified by the line segment, the center of the axis is located
     * at the first end point that defines the segment.
     *
     * @param p Point to project
     * @returns Single floating point number that determines the position of the projection on the
     *          axis with respect to the first end point
     */
    double project1D(const Point &p) const;

    /**
     * @param s Query segment
     * @param margin Margin of error required in order to consider the two line segments'
     *               intersection an actual intersection (should be much smaller than 1)
     * @returns true if the line segment is intersected by s, false otherwise
     */
    bool intersects(const LineSegment &s, double margin = 0) const;

    /**
     * @param center Center of the query circle
     * @param radius Radius of the query circle
     * @returns true if the line segment intersects the circle, false otherwise
     */
    bool intersectsCircle(const Point &center, double radius) const;

    double intersection1D(const LineSegment &s) const;

    /**
     * @param s The line segment to intersect to
     * @returns The point at which the lines identified by the two line segments intersect. Note:
     *          this may be outside of both segments
     */
    Point intersection(const LineSegment &s) const;

    /**
     * @param s Line segment to merge with this
     * @returns A line segment which is the weighted merge of this segment and s
     */
    LineSegment merge(const LineSegment &s) const;
    /**
     * Merge to line segment specified by s, modifies current object
     * @param s Line segment to merge with this
     */
    void mergeInPlace(const LineSegment &s);
    void mergeInPlaceProjection(const LineSegment &s);

    bool atRight(const Point &p) const;
    bool atLeft(const Point &p) const;


    LineSegment portion(double from, double to) const;
    LineSegment portionNormalized(double from, double to) const;

    /**
     * @returns The endpoint with the minimum x coordinate
     */
    const Point &vertexMinX() const;
    /**
     * @returns The endpoint with the minimum y coordinate
     */
    const Point &vertexMinY() const;
    /**
     * @returns The endpoint with the maximum x coordinate
     */
    const Point &vertexMaxX() const;
    /**
     * @returns The endpoint with the maximum y coordinate
     */
    const Point &vertexMaxY() const;

    Eigen::Vector2d lineVector() const;
    Eigen::Vector4d vector() const;

    bool operator==(const LineSegment &s) const;
    bool operator!=(const LineSegment &s) const;

    bool operator<(const LineSegment& s) const;

    operator QLineF() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    friend LoggerStream &operator<<(LoggerStream &stream, const LineSegment &line);
    friend LoggerStream &operator<<(LoggerStream &stream, const LineSegment *line);

protected:
    Point p1v, p2v;
};

inline LineSegment::LineSegment() : p1v(), p2v()
{
}

inline LineSegment::LineSegment(const Point &p1, const Point &p2) : p1v(p1), p2v(p2)
{
}

inline LineSegment::LineSegment(double x1, double y1, double x2, double y2)
    : p1v(x1, y1), p2v(x2, y2)
{
}

inline LineSegment::LineSegment(const LineSegment &s) : Data::Serializable(), p1v(s.p1v), p2v(s.p2v)
{
}

inline const Point &LineSegment::p1() const
{
    return p1v;
}

inline const Point &LineSegment::p2() const
{
    return p2v;
}

inline void LineSegment::setP1(const Point &p1)
{
    p1v = p1;
}

inline void LineSegment::setP2(const Point &p2)
{
    p2v = p2;
}

inline double LineSegment::distance2(const Point &p) const
{
    /*
               ( det[ l1-p l2-p ] )^2
        d^2 = ------------------------
                  || l2 - l1 ||^2
    */

    const double l1px = p1().x() - p.x();
    const double l1py = p1().y() - p.y();
    const double l2px = p2().x() - p.x();
    const double l2py = p2().y() - p.y();
    double det = l1px * l2py - l1py * l2px;

    return det * det / p2v.distance2(p1v);
}

inline double LineSegment::distance(const Point &p) const
{
    return std::sqrt(distance2(p));
}

inline double LineSegment::x1() const
{
    return p1v.x();
}

inline double LineSegment::y1() const
{
    return p1v.y();
}

inline double LineSegment::x2() const
{
    return p2v.x();
}

inline double LineSegment::y2() const
{
    return p2v.y();
}

inline double LineSegment::dx() const
{
    return p2v.x() - p1v.x();
}

inline double LineSegment::dy() const
{
    return p2v.y() - p1v.y();
}

inline Point LineSegment::centroid() const
{
    return (p1v + p2v) / 2;
}

inline double LineSegment::length2() const
{
    return p1v.distance2(p2v);
}

inline double LineSegment::length() const
{
    return p1v.distance(p2v);
}

inline double LineSegment::angle() const
{
    return (p2v - p1v).angle();
}

inline double LineSegment::m() const
{
    return dy() / dx();
}

inline double LineSegment::q() const
{
    return p1v.y() - m() * p1v.x();
}

inline Point LineSegment::project(const Point &p) const
{
    /* TODO: Not safe against vertical segments */
    /*double mv = m(), qv = q();
    double projx = (p.x() + mv * p.y() - mv * qv) / (1 + mv * mv);
    double projy = mv * projx + qv;*/

    /* a * x + b * x + c = 0 */
    double a = - dy(), b = dx();
    double c = - a * p1v.x() - b * p1v.y();

    return project(a, b, c, p);
}

inline Point LineSegment::project(double a, double b, double c, const Point &p)
{
    double den = a * a + b * b;
    double projx = - (a * c - b * b * p.x() + a * b * p.y()) / den;
    double projy = - (b * c + a * b * p.x() - a * a * p.y()) / den;

    return Point(projx, projy);
}

inline bool LineSegment::intersects(const LineSegment &s, double margin) const
{
    /*
        Express the two segments as a convex combination of the end points, then solve against the
        two parameters l1 and l2:
            (1 - l1) * P1 + l1 * P2 = (1 - l2) * Q1 + l2 * Q2
        If 0 <= l1, l2 <= 1, then the two segments intersect.
        Solution obtained with Mathematica with the command:
            FullSimplify[Solve[(1 - l1) {p1x, p1y} + l1 {p2x, p2y} ==
                    (1 - l2) {q1x, q1y} + l2 {q2x, q2y}, {l1, l2}]]
    */
    double p1x  =   p1v.x(), p1y =   p1v.y();
    double p2x  =   p2v.x(), p2y =   p2v.y();
    double q1x  = s.p1v.x(), q1y = s.p1v.y();
    double q2x  = s.p2v.x(), q2y = s.p2v.y();

    double den  = (p1y - p2y) * (q1x - q2x) - (p1x - p2x) * (q1y - q2y);
    double num1 = q1x * q2y + p1y * ( q2x - q1x) + p1x * (q1y - q2y) - q1y * q2x;
    double num2 = p2x * q1y + p1y * (-p2x + q1x) + p1x * (p2y - q1y) - p2y * q1x;

    /* If the denominator is zero the two segments are parallel */
    if(almostEqual(den, 0)) return false;

    double l1   = - num1 / den;
    double l2   = num2 / den;

    return l1 >= margin && l1 <= 1 - margin && l2 >= margin && l2 <= 1 - margin;
}

inline bool LineSegment::intersectsCircle(const Point &center, double radius) const
{
    const double r2 = radius * radius;

    /* One of the endpoints is in the circle */
    if(p1v.distance2(center) <= r2 || p2v.distance2(center) <= r2)
        return true;

    /*
        Otherwise express the segment as a convex combination of the two endpoints and intersect
        it with the circle, then check:
            1. If a solution doesn't exist, then the geometric objects don't intersect
            2. If they do, the geometric objects intersect only if the 0 <= l <= 1
        Solved with Mathematica using the following command:
            FullSimplify[Solve[(l x1 + (1 - l) x2 - xc)^2 + (l y1 + (1 - l) y2 - yc)^2 == r^2, l]]
     */
    const double x1 = p1v.x(),      y1 = p1v.y();
    const double x2 = p2v.x(),      y2 = p2v.y();
    const double xc = center.x(),   yc = center.y();
    const double l2 = square(x1 - x2) + square(y1 - y2);

    const double tmp = xc * y1 + x1 * y2 - xc * y2 - x2 * (y1 - yc) - x1 * yc;
    const double det = r2 * l2 - square(tmp);
    /* This value goes under a square root, if it is negative then a solution doesn't exist */
    if(det < 0) return false;

    const double rdet = std::sqrt(det);
    const double part = x2 * x2 + x1 * xc - x2 * (x1 + xc) - y1 * y2 + y2 * y2 + y1 * yc - y2 * yc;

    /* Solutions, scaled by l2 to avoid two divisions */
    const double s1 = part + rdet, s2 = part - rdet;

    return (s1 >= 0 && s1 <= l2) || (s2 >= 0 && s1 <= l2);
}

inline double LineSegment::intersection1D(const LineSegment &s) const
{
    return project1D(intersection(s));
}

inline Point LineSegment::intersection(const LineSegment &s) const
{
    /*
    double m1 = m(), m2 = s.m();
    double p1x  =   p1v.x(), p1y =   p1v.y();
    double q1x  = s.p1v.x(), q1y = s.p1v.y();

    double x = (m1 * p1x - p1y - m2 * q1x + q1y) / (m1 - m2);
    return Point(x, m1 * (x - p1x) + p1y);
    */
    const double p1x  =   p1v.x(), p1y =   p1v.y();
    const double p2x  =   p2v.x(), p2y =   p2v.y();
    const double q1x  = s.p1v.x(), q1y = s.p1v.y();
    const double q2x  = s.p2v.x(), q2y = s.p2v.y();
    const double tmp1 = p1y - p2y;
    const double tmp2 = q1y - q2y;
    const double den = (-((q1x - q2x)*tmp1) + (p1x - p2x)*tmp2);
    if(almostEqual(den, 0)) return Point(INFINITY, INFINITY);
    const double tmp3 = 1/den;
    const double tmp4 = q1y*q2x;
    const double tmp5 = -(q1x*q2y);
    return Point(
            tmp3*(-(p2x*q1y*q2x) + p1y*p2x*(-q1x + q2x) + p2x*q1x*q2y +
                    p1x*(p2y*q1x - p2y*q2x + tmp4 + tmp5)),
            tmp3*(p2y*(-(q1y*q2x) + q1x*q2y + p1x*tmp2) +
                    p1y*(p2x*(-q1y + q2y) + tmp4 + tmp5)));

}

inline double LineSegment::project1D(const Point &p) const
{
    /*
        Express the line identified by the segment in parametric form as a normalised linear
        combination of the two end points. It is normalised in such a way that when the parameter
        is zero the formula yields the first end point, when it is equal to the length it yields
        the second end point:
           | x |         1         /                                 \
           |   | = ------------- * | (||P1 - P2|| - l) * P1 + l * P2 |
           | y |    ||P1 - P2||    \                                 /
        Then intersect the parametric line with the perpendicular line passing through the point
        p = (x0, y0):
                        P2x - P1x
            y - y0 = - ----------- * (x - x0)
                        P2y - P1y
        Then solve against l; l is the value we seek
    */
    double p1x = p1v.x(), p1y = p1v.y();
    double p2x = p2v.x(), p2y = p2v.y();
    double x0  = p.x()  , y0  = p.y();

    return (p1x * p1x + p1y * p1y + p2x * x0 + p2y * y0
            - p1x * (p2x + x0) - p1y * (p2y + y0)) / length();
}

inline LineSegment LineSegment::merge(const LineSegment &s) const
{
    double l1 = length(), l2 = s.length();
    double lsum = l1 + l2;

    /* Determine weighted centroid */
    double cx = (l1 * (p1v.x() + p2v.x()) + l2 * (s.p1v.x() + s.p2v.x())) / (2 * lsum);
    double cy = (l1 * (p1v.y() + p2v.y()) + l2 * (s.p1v.y() + s.p2v.y())) / (2 * lsum);

    double tmerge = weightedLineAngle(angle(), s.angle(), l1, l2);

    /*
        Determine the 1D projection of all endpoints on the reference frame centered in the
        weighted centroid, with rotation given by tmerge
    */
    double cs = std::cos(tmerge), sn = std::sin(tmerge);
    double p1 = (p1v.x() - cx) * cs + (p1v.y() - cy) * sn;
    double p2 = (p2v.x() - cx) * cs + (p2v.y() - cy) * sn;
    double q1 = (s.p1v.x() - cx) * cs + (s.p1v.y() - cy) * sn;
    double q2 = (s.p2v.x() - cx) * cs + (s.p2v.y() - cy) * sn;

    /* Find minimum and maximum */
    double pmin = min(p1, p2, q1, q2);
    double pmax = max(p1, p2, q1, q2);

    /* Re-express in original reference frame and return segment */
    return LineSegment(cx + pmin * cs, cy + pmin * sn, cx + pmax * cs, cy + pmax * sn);
}

inline void LineSegment::mergeInPlaceProjection(const LineSegment &s)
{
    const double ct = std::cos(angle()), st = std::sin(angle());
    double len = length();
    double proj1 = project1D(s.p1()), proj2 = project1D(s.p2());
    double np1 = min(0., proj1, proj2), np2 = max(len, proj1, proj2);

    p2v = Point(p1v.x() + np2 * ct, p1v.y() + np2 * st);
    p1v = Point(p1v.x() + np1 * ct, p1v.y() + np1 * st);

    //const Point oldp1v = p1v, oldp2v = p2v;
    //p1v = (1 - np1) * oldp1v + np1 * oldp2v;
    //p1v = (1 - np2) * oldp1v + np2 * oldp2v;
}

inline void LineSegment::mergeInPlace(const LineSegment &s)
{
    LineSegment mm = merge(s);
    p1v = mm.p1v;
    p2v = mm.p2v;
}

inline bool LineSegment::atRight(const Point &p) const
{
    return (p1v.x() - p.x()) * (p2v.y() - p.y()) - (p1v.y() - p.y()) * (p2v.x() - p.x()) <= 0;
}

inline bool LineSegment::atLeft(const Point &p) const
{
    return (p1v.x() - p.x()) * (p2v.y() - p.y()) - (p1v.y() - p.y()) * (p2v.x() - p.x()) >= 0;
}

inline LineSegment LineSegment::portion(double from, double to) const
{
    const double len = length();
    return portionNormalized(from / len, to / len);
}

inline LineSegment LineSegment::portionNormalized(double from, double to) const
{
    return LineSegment(
                    (1 - from) * p1() + from * p2(),
                    (1 - to  ) * p1() + to   * p2());
}

inline const Point &LineSegment::vertexMinX() const
{
	return p1v.x() < p2v.x() ? p1v : p2v;
}

inline const Point &LineSegment::vertexMinY() const
{
	return p1v.y() < p2v.y() ? p1v : p2v;
}

inline const Point &LineSegment::vertexMaxX() const
{
	return p1v.x() > p2v.x() ? p1v : p2v;
}

inline const Point &LineSegment::vertexMaxY() const
{
	return p1v.y() > p2v.y() ? p1v : p2v;
}

inline double LineSegment::psiProjection(const Point &p) const
{
    const double a = alpha();
    return p.y() * std::cos(a) - p.x() * std::sin(a);
}

inline double LineSegment::alpha() const
{
    return wrap(angle() + M_PI_2);
}

inline double LineSegment::rho() const
{
    const double a = alpha();
    return p1v.x() * std::cos(a) + p1v.y() * std::sin(a);
}

inline double LineSegment::psi1() const
{
    return psiProjection(p1v);
}

inline double LineSegment::psi2() const
{
    return psiProjection(p2v);
}

inline Eigen::Vector2d LineSegment::lineVector() const
{
    return Eigen::Vector2d(alpha(), rho());
}

inline Eigen::Vector4d LineSegment::vector() const
{
    return Eigen::Vector4d(alpha(), rho(), psi1(), psi2());
}

inline bool LineSegment::operator==(const LineSegment &s) const
{
    return p1v == s.p1v && p2v == s.p2v;
}

inline bool LineSegment::operator!=(const LineSegment &s) const
{
    return p1v != s.p1v || p2v != s.p2v;
}

inline LineSegment::operator QLineF() const
{
    return QLineF(p1v, p2v);
}

inline bool LineSegment::operator <(const LineSegment& s)const{
    if(p1v.x()==s.p1v.x()){
        if(p1v.y()==s.p1v.y()){
            if(p2v.x()==s.p2v.x()){
                return p2v.y()<s.p2v.y();
            }
            return p2v.x()<s.p2v.x();
        }
        return p1v.y()<s.p1v.y();
    }
    return p1v.x()<s.p1v.x();
}

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* LINESEGMENT_H */
