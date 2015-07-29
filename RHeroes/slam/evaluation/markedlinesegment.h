/*
 * markedlinesegment.h
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef MARKEDLINESEGMENT_H_
#define MARKEDLINESEGMENT_H_

#include "slam/geometry/linesegment.h"
#include "slam/geometry/segmentscan.h"
#include <QList>

namespace SLAM {
namespace Evaluation {

class MarkedLineSegment : public Geometry::LineSegment {
public:
    MarkedLineSegment(Geometry::SegmentScan *scan, int idx);
    MarkedLineSegment(const MarkedLineSegment &s);

    double weight() const { return w; }
    MarkedLineSegment merge(const MarkedLineSegment &s) const;
    bool contains(const Geometry::SegmentScan *scan, int idx) const;

    MarkedLineSegment &operator=(const MarkedLineSegment &s);

protected:
    MarkedLineSegment();

private:
    QList<std::pair<Geometry::SegmentScan *, int> > components;
    double w;
};

inline MarkedLineSegment::MarkedLineSegment() :
        LineSegment(), w(0)
{
}

inline MarkedLineSegment::MarkedLineSegment(Geometry::SegmentScan *scan, int idx) :
        LineSegment(scan->getSegments().at(idx)), w(scan->getSegments().at(idx).length())
{
    components.append(std::make_pair(scan, idx));
}

inline MarkedLineSegment::MarkedLineSegment(const MarkedLineSegment &s) :
        LineSegment(s), components(s.components), w(s.w)
{
}

inline MarkedLineSegment &MarkedLineSegment::operator=(const MarkedLineSegment &s)
{
    static_cast<LineSegment &>(*this) = s;
    components = s.components;
    w = s.w;
    return *this;
}

inline bool MarkedLineSegment::contains(const Geometry::SegmentScan *scan, int idx) const
{
    //ldbg << &scan << "," << idx << ": " << components << endl;
    return components.contains(std::make_pair(const_cast<Geometry::SegmentScan *>(scan), idx));
}

inline MarkedLineSegment MarkedLineSegment::merge(const MarkedLineSegment &s) const
{
    double w1 = w, w2 = s.w;
    double wsum = w1 + w2;

    /* Determine weighted centroid */
    double cx = (w1 * (p1v.x() + p2v.x()) + w2 * (s.p1v.x() + s.p2v.x())) / (2 * wsum);
    double cy = (w1 * (p1v.y() + p2v.y()) + w2 * (s.p1v.y() + s.p2v.y())) / (2 * wsum);

    double tmerge = weightedLineAngle(angle(), s.angle(), w1, w2);

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
    LineSegment m(cx + pmin * cs, cy + pmin * sn, cx + pmax * cs, cy + pmax * sn);

    MarkedLineSegment ret;
    static_cast<LineSegment &>(ret) = m;
    ret.components = components;
    ret.components.append(s.components);
    ret.w = wsum;
    return ret;
}

} /* namespace Evaluation */
} /* namespace SLAM */

#endif /* MARKEDLINESEGMENT_H_ */
