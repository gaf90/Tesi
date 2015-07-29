/*
 * segmentscan.h
 *
 *  Created on: 24/gen/2012
 *      Author: Mladen Mazuran
 */

#ifndef SEGMENTSCAN_H_
#define SEGMENTSCAN_H_

#include "pointscan.h"
#include "slam/geometry/rototranslation.h"
#include "slam/geometry/uncertainlinesegment.h"
#include "slam/geometry/frontier.h"
#include "slam/geometry/visibilitypolygon.h"
#include "slam/support/indexset.h"
#include "slam/support/alignedvector.h"
#include <QList>

namespace SLAM {
namespace Geometry {

class SegmentScan
{
public:

    enum {
        SplitAndMergeInterpolation      = 0x01,
        RANSACInterpolation             = 0x02,

        MedianFiltering                 = 0x04,

        WeightedLineFitting             = 0x08,
        MedianLineFitting               = 0x10,
        TotalLeastSquaresFitting        = 0x20,
        TotalLeastTrimmedSquaresFitting = 0x40,
        RepeatedMedianLineFitting       = 0x80
    };

    SegmentScan();
    SegmentScan(const PointScan &pscan,
                int options = SplitAndMergeInterpolation | MedianLineFitting);
    SegmentScan(const AlignedVector<Geometry::UncertainLineSegment> &segments,
                const QList<Geometry::Frontier> &frontiers);
    SegmentScan(const SegmentScan &sscan);
    virtual ~SegmentScan();

    const inline AlignedVector<Geometry::UncertainLineSegment> &getSegments() const { return segments; }
    const inline QList<Geometry::Frontier> &getFrontiers() const { return frontiers; }

    const Geometry::VisibilityPolygon &toPolygon() const;

    friend SegmentScan operator*(const Geometry::Rototranslation &rt, const SegmentScan &scan);

private:
    struct LineSegmentTuple {
        inline LineSegmentTuple(const UncertainLineSegment &segment, const Support::IndexSet &index) :
                    segment(segment), index(index) {}

        inline bool operator<(const LineSegmentTuple &lst) const {
            return index.indexBegin() < lst.index.indexBegin();
        }

        inline int startIndex() const { return index.indexBegin(); }
        inline int endIndex()   const { return index.indexEnd();   }

        Geometry::UncertainLineSegment segment;
        Support::IndexSet index;
    };

    void findLinesSplitAndMerge(const PointScan &pscan);
#if 0
    void findFrontiersSplitAndMerge(const PointScan &pscan);
    void findPolygonSplitAndMerge();
#endif
    void split(const PointScan &pscan, const Support::IndexSet &idx);
    void merge(const PointScan &pscan);


    void findLinesRANSAC(const PointScan &pscan);
    void findFrontiersAndPolygon(const PointScan &pscan);
    void findLinesSingleClusterRANSAC(
            const PointScan &pscan, const Support::IndexSet &cluster);
    std::pair<int, bool> sweepNext(const QList<int> &open, int i) const;
    Point findMaxIntersection(const QList<int> &open, const LineSegment &jump) const;

    const Geometry::UncertainLineSegment fittingFunction(
            const PointScan &scan, const Support::IndexSet &idx);

private:
    AlignedVector<Geometry::UncertainLineSegment> segments;
    QList<Geometry::Frontier> frontiers;
    AlignedVector<LineSegmentTuple> clusterSupport, support;
    Geometry::VisibilityPolygon polygon;
    Geometry::Point center;
    int options;
};

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* SEGMENTSCAN_H_ */
