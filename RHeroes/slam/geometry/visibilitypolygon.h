/*
 * visibilitypolygon.h
 *
 *  Created on: 08/mag/2012
 *      Author: Mladen Mazuran
 */

#ifndef VISIBILITYPOLYGON_H_
#define VISIBILITYPOLYGON_H_

#include <QList>
#include "linesegment.h"
#include "rototranslation.h"
#include "slam/utilities.h"
#include "slam/support/topvalues.h"

namespace SLAM {
namespace Geometry {

class SegmentScan;

class VisibilityPolygon
{
public:
    VisibilityPolygon();
    VisibilityPolygon(const VisibilityPolygon &v);
    virtual ~VisibilityPolygon();

    bool contains(const Point &p) const;
    bool contains(const LineSegment &s) const;
    bool containsEndPoint(const LineSegment &s) const;
    QList<Point> vertices() const;

    LineSegment lineDifference(const LineSegment &s) const;

    VisibilityPolygon &operator=(const VisibilityPolygon &v);

    friend VisibilityPolygon operator*(const Rototranslation &rt, const VisibilityPolygon &v);
    friend LoggerStream &operator<<(LoggerStream &stream, const VisibilityPolygon &v);
    friend LoggerStream &operator<<(LoggerStream &stream, const VisibilityPolygon *v);
    friend class Geometry::SegmentScan;

private:
    QList<LineSegment> edges;
    double maxx, maxy;
};

inline VisibilityPolygon::VisibilityPolygon() :
    maxx(0), maxy(0)
{
}

inline VisibilityPolygon::VisibilityPolygon(const VisibilityPolygon &v) :
    edges(v.edges), maxx(v.maxx), maxy(v.maxy)
{
}

inline VisibilityPolygon::~VisibilityPolygon()
{
}

inline bool VisibilityPolygon::contains(const Point &p) const
{
    int count = 0;
    LineSegment tester(p.x(), p.y(), maxx + 1, maxy + 1);
    fforeach(const LineSegment &edge, edges) {
        if(tester.intersects(edge))
            count++;
    }
    return count % 2 == 1;
}

inline bool VisibilityPolygon::contains(const LineSegment &s) const
{
    return contains(s.p1()) && contains(s.p2());
}

inline bool VisibilityPolygon::containsEndPoint(const LineSegment &s) const
{
    return contains(s.p1()) || contains(s.p2());
}


inline QList<Point> VisibilityPolygon::vertices() const
{
    QList<Point> ret;
    if(edges.size() <= 0) return ret;
    ret << edges.first().p1();
    for(int i = 0; i < edges.size(); i++) {
        ret << edges[i].p2();
    }
    return ret;
}

inline LineSegment VisibilityPolygon::lineDifference(const LineSegment &s) const
{
#if 0
    const double len = s.length();
    double lmin = 0, lmax = len;
    bool hasIntersection = false;
    //if(contains(s))
    //    return LineSegment(0,0,0,0);
    fforeach(const LineSegment &edge, edges) {
        if(s.intersects(edge)) {
            const double int1d = s.intersection1D(edge);

            /* The segment is oriented, detect which end point of s is "inside"
               (not necessarily, but we don't expect complex geometries) */
            const double det1 = (edge.p1().x() - s.p1().x()) * (edge.p2().y() - s.p1().y()) -
                                (edge.p1().y() - s.p1().y()) * (edge.p2().x() - s.p1().x());
            if(det1 >= 0) {
                /* s.p1() is "inside" */
                lmin = max(lmin, int1d);
            } else {
                /* s.p2() is "inside" */
                lmax = min(lmax, int1d);
            }

            hasIntersection = true;
        }
    }
    if((!hasIntersection && containsEndPoint(s)) || lmax <= lmin) {
        return LineSegment(0,0,0,0);
    } else  {
        lmin /= len;
        lmax /= len;

        return LineSegment(
                (1 - lmin) * s.p1() + lmin * s.p2(),
                (1 - lmax) * s.p1() + lmax * s.p2());
    }
#endif
    const double len = s.length();
    QList<double> starts, ends;
    bool hasIntersection = false;
    //if(contains(s))
    //    return LineSegment(0,0,0,0);
    fforeach(const LineSegment &edge, edges) {
        if(s.intersects(edge)) {
            const double int1d = s.intersection1D(edge);

            //ldbg << "Graphics[{" << s << "," << edge << "}]" << endl;

            /* The segment is oriented, detect which end point of s is "inside"
               (not necessarily, but we don't expect complex geometries) */
            //const double det1 = (edge.p1().x() - s.p1().x()) * (edge.p2().y() - s.p1().y()) -
            //                    (edge.p1().y() - s.p1().y()) * (edge.p2().x() - s.p1().x());
            if(edge.atLeft(s.p1())) { //if(det1 >= 0) {
                /* s.p1() is "inside" */
                starts.append(int1d);
            } else {
                /* s.p2() is "inside" */
                ends.append(int1d);
            }

            hasIntersection = true;
        }
    }
    if(!hasIntersection && containsEndPoint(s)) {
        return LineSegment(0,0,0,0);
    } else  {
        qSort(starts);
        qSort(ends);
        if(starts.size() > 0) {
            if(ends.size() > 0) {
                if(ends.first() < starts.first()) starts.prepend(0);
                if(starts.last() > ends.last()) ends.append(len);

                Support::TopValues<1> tv;
                for(int i = 0; i < min(starts.size(), ends.size()); i++) {
                    tv.add(- (ends[i] - starts[i]), i);
                }
                const int i = tv.value();
                return s.portion(starts[i], ends[i]);
            } else {
                return s.portion(starts.last(), len);
            }
        } else if(ends.size() > 0) {
            return s.portion(0, ends.first());
        } else {
            return s;
        }
    }
}

inline VisibilityPolygon &VisibilityPolygon::operator=(const VisibilityPolygon &v)
{
    edges = v.edges;
    maxx = v.maxx;
    maxy = v.maxy;
    return *this;
}

inline VisibilityPolygon operator*(const Rototranslation &rt, const VisibilityPolygon &v)
{
    VisibilityPolygon vp;
    fforeach(const LineSegment &l, v.edges) {
        LineSegment rtl = rt * l;
        vp.edges.append(rtl);
        vp.maxx = max(vp.maxx, rtl.p1().x(), rtl.p2().x());
        vp.maxy = max(vp.maxy, rtl.p1().y(), rtl.p2().y());
    }
    return vp;
}

inline LoggerStream &operator<<(LoggerStream &stream, const VisibilityPolygon &v)
{
    if(v.edges.size() <= 0)
        return stream << "Polygon[{}]";
    stream << "Polygon[{" << v.edges.first().p1() << ",";
    for(int i = 0; i < v.edges.size(); i++) {
        stream << v.edges[i].p2();
        if(i < v.edges.size() - 1)
            stream << ",";
    }
    return stream << "}]";
}

inline LoggerStream &operator<<(LoggerStream &stream, const VisibilityPolygon *v)
{
    return stream << *v;
}

} /* namespace Geometry */
} /* namespace SLAM */
#endif /* VISIBILITYPOLYGON_H_ */
