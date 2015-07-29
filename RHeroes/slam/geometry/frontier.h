/*
 * frontier.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef FRONTIER_H_
#define FRONTIER_H_

#include "linesegment.h"
#include "slam/pathnode.h"
#include <QObject>

#define FRONTIER_THRESHOLD 2.5

namespace SLAM {
namespace Geometry {

class Frontier : public LineSegment
{
public:
    Frontier();
    Frontier(const Point &p1, const Point &p2, PathNode *from = NULL);
    Frontier(double x1, double y1, double x2, double y2, PathNode *from = NULL);
    Frontier(const LineSegment &s);
    Frontier(const Frontier &f);

    PathNode *visibleFrom();
    const PathNode *visibleFrom() const;

    void setVisibility(PathNode *node);

    Frontier &operator=(const LineSegment &s);

    bool operator==(const Frontier& f)const;

    friend LoggerStream &operator<<(LoggerStream &stream, const Frontier *front);

private:
    PathNode *from;
};

inline Frontier::Frontier() : LineSegment() {}

inline Frontier::Frontier(const Point &p1, const Point &p2, PathNode *from) :
    LineSegment(p1, p2), from(from)
{
}

inline Frontier::Frontier(double x1, double y1, double x2, double y2, PathNode *from)
    : LineSegment(x1, y1, x2, y2), from(from)
{
}

inline Frontier::Frontier(const LineSegment &s) : LineSegment(s), from(NULL)
{
}

inline Frontier::Frontier(const Frontier &f) : LineSegment(f), from(f.from)
{
}

inline PathNode *Frontier::visibleFrom()
{
    return from;
}

inline const PathNode *Frontier::visibleFrom() const
{
    return from;
}

inline void Frontier::setVisibility(PathNode *node)
{
    from = node;
}

inline Frontier &Frontier::operator=(const LineSegment &s)
{
    setP1(s.p1());
    setP2(s.p2());
    return *this;
}

inline uint qHash(const Frontier &f) {
    union {
        double d;
        quint64 u;
    } x1u, y1u, x2u, y2u;
    x1u.d = f.x1(); y1u.d = f.y1();
    x2u.d = f.x2(); y2u.d = f.y2();
    quint64 x1 = x1u.u * 17, y1 = y1u.u * 29;
    quint64 x2 = x2u.u * 43, y2 = y2u.u * 19;
    quint64 twoHash = x1 ^ y1 ^ x2 ^ y2;

    return (twoHash & 0xFFFFFFFFLU) ^ (twoHash >> 32);
}

inline bool Frontier::operator ==(const Frontier& f)const{
    return this->centroid().distance(f.centroid())<FRONTIER_THRESHOLD;
}

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* FRONTIER_H_ */
