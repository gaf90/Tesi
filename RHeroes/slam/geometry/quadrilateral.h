/*
 * quadrilateral.h
 *
 *  Created on: 09/mag/2012
 *      Author: Mladen Mazuran
 */

#ifndef QUADRILATERAL_H_
#define QUADRILATERAL_H_

namespace SLAM {
namespace Geometry {

class Quadrilateral
{
public:
    Quadrilateral(const Point &p1v, const Point &p2v, const Point &p3v, const Point &p4v);
    virtual ~Quadrilateral();

    const Point &p1() const;
    const Point &p2() const;
    const Point &p3() const;
    const Point &p4() const;

    bool contains(const Point &p) const;

private:
    Point inverseAffine(const Point &q01, const Point &q10, const Point &p) const;

    Point p1v, p2v, p3v, p4v;
};

inline Quadrilateral::Quadrilateral(
        const Point &p1v, const Point &p2v, const Point &p3v, const Point &p4v) :
        p1v(p1v), p2v(p2v), p3v(p3v), p4v(p4v)
{
}

inline Quadrilateral::~Quadrilateral()
{
}

inline const Point &Quadrilateral::p1() const
{
    return p1v;
}

inline const Point &Quadrilateral::p2() const
{
    return p2v;
}

inline const Point &Quadrilateral::p3() const
{
    return p3v;
}

inline const Point &Quadrilateral::p4() const
{
    return p4v;
}

inline Point Quadrilateral::inverseAffine(
        const Point &q01, const Point &q10, const Point &p) const
{
    double den = q01.y() * q10.x() - q01.x() * q10.y();
    double x1 = (q10.x() * p.y() - q10.y() * p.x()) / den; /* New x coordinate */
    double y1 = (q01.y() * p.x() - q01.x() * p.y()) / den; /* New y coordinate */

    return Point(x1, y1);
}

inline bool Quadrilateral::contains(const Point &p) const
{
    /*
       Map the quadrilateral to a quadrilateral with three vertices in (0,0), (0,1), (1,0) and
       then apply a projective transformation to map the last vertex to (1,1).
       The point p is contained in the quadrilateral if the final coordinates are greater than 0
       and smaller than 1
    */

    Point q01 = p2v - p1v;
    Point q10 = p4v - p1v;

    Point a  = inverseAffine(q01, q10, p3v - p1v);
    Point p1 = inverseAffine(q01, q10, p - p1v);

    double a0  = a.x(),  a1  = a.y();
    double p1x = p1.x(), p1y = p1.y();

    /* Apply projective transform */
    double den = a0 * a1 + a1 * (a1 - 1) * p1x + a0 * (a0 - 1) * p1y;
    double x   = (a1 * (a0 + a1 - 1) * p1x) / den;
    double y   = (a0 * (a0 + a1 - 1) * p1y) / den;

    return x > 0 && x < 1 && y > 0 && y < 1;
}

inline LoggerStream &operator<<(LoggerStream &stream, const Quadrilateral &q)
{
    return stream << "Polygon[{" <<
            q.p1() << "," << q.p2() << "," << q.p3() << "," << q.p4() << "}]";
}

} /* namespace Geometry */
} /* namespace SLAM */
#endif /* QUADRILATERAL_H_ */
