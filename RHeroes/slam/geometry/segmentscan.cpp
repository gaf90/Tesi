/*
 * segmentscan.cpp
 *
 *  Created on: 24/gen/2012
 *      Author: Mladen Mazuran
 */

#include "segmentscan.h"
#include "clustering.h"
#include "linefitting.h"
#include "slam/utilities.h"
#include "shared/random.h"
#include <cmath>
#include <iostream>

namespace SLAM {
namespace Geometry {

using namespace Support;

SegmentScan::SegmentScan()
{
}

SegmentScan::SegmentScan(const PointScan &pscan, int options) : options(options)
{
    if(!(options & (
            MedianLineFitting | TotalLeastSquaresFitting |
            TotalLeastTrimmedSquaresFitting |
            RepeatedMedianLineFitting | WeightedLineFitting))) {
        this->options |= MedianLineFitting;
    }
    if(options & MedianFiltering) {
        PointScan scan = PointScan::medianFilter(pscan, 5);
        if(options & SplitAndMergeInterpolation) {
            findLinesSplitAndMerge(scan);
        } else {
            findLinesRANSAC(scan);
        }
        findFrontiersAndPolygon(scan);
    } else {
        if(options & SplitAndMergeInterpolation) {
            findLinesSplitAndMerge(pscan);
        } else {
            findLinesRANSAC(pscan);
        }
        findFrontiersAndPolygon(pscan);
    }
}


SegmentScan::SegmentScan(
            const AlignedVector<UncertainLineSegment> &segments,
            const QList<Frontier> &frontiers) :
    segments(segments), frontiers(frontiers)
{
}


SegmentScan::SegmentScan(const SegmentScan &sscan) :
    segments(sscan.segments), frontiers(sscan.frontiers)
{
}

SegmentScan::~SegmentScan()
{
}

inline const Geometry::UncertainLineSegment SegmentScan::fittingFunction(
            const PointScan &scan, const Support::IndexSet &idx)
{
    if(options & MedianLineFitting) {
        return medianLineFit(scan, idx);
    } else if(options & TotalLeastSquaresFitting) {
        return totalLeastSquaresLineFit(scan, idx);
    } else if(options & TotalLeastTrimmedSquaresFitting) {
        return totalLeastTrimmedSquaresLineFit(scan, idx);
    } else if(options & RepeatedMedianLineFitting) {
        return repeatedMedianLineFit(scan, idx);
    } else { /* if(options & WeightedLineFitting) */
        return weightedLineFit(scan, idx);
    }
}

static inline double findMaximumDistance(
    const PointScan &scan, const IndexSet &idx, const LineSegment &segment,
    int *pointIndex)
{
    double maxDist = 0;
    int maxDistIdx = -1;

    fforeach(int i, idx) {
        double dist = segment.distance2(scan[i]);
        if(dist > maxDist) {
            maxDist = dist;
            maxDistIdx = i;
        }
    }

    if(pointIndex != NULL)
        *pointIndex = maxDistIdx;
    return maxDist;
}

void SegmentScan::findLinesSplitAndMerge(const PointScan &pscan)
{
    const QList<IndexSet> clusters = findClustersAggressive(pscan);

    for(int i = 0; i < clusters.size(); i++) {
        split(pscan, clusters[i]);
        merge(pscan);
    }
    qSort(support);
}

void SegmentScan::split(const PointScan &pscan, const IndexSet &idx)
{
    /* No segment to fit */
    if(idx.size() < 2) return;

    /* Fit line segment between first and last point
       (only do least squares fitting when splitting has ended) */
    LineSegment segment(pscan[idx.indexBegin()], pscan[idx.indexEnd() - 1]);

#if 0
    /* Minimal segment */
    if(idx.size() == 2) {
        const UncertainLineSegment usegment = fittingFunction(pscan, idx);
        const LineSegmentTuple tuple = { usegment, idx };
        clusterSupport.append(tuple);
        return;
    }
#endif

    /* Find most distant point from the line */
    int maxDistIdx;
    double maxDist =
            findMaximumDistance(pscan, idx, segment, &maxDistIdx);

    /* Split recursively if necessary, otherwise add the segment to the list */
    if(maxDist > SQUARE(Config::SLAM::splitThreshold)) {
        //ldbg << "split 1: " << idx.firstHalfSplit(maxDistIdx) << endl;
        //ldbg << "split 2: " << idx.secondHalfSplit(maxDistIdx) << endl;
        split(pscan, idx.firstHalfSplit(maxDistIdx));
        split(pscan, idx.secondHalfSplit(maxDistIdx));
    } else {
        const UncertainLineSegment usegment = fittingFunction(pscan, idx);
        const LineSegmentTuple tuple(usegment, idx);
        clusterSupport.append(tuple);
    }

}

void SegmentScan::merge(const PointScan &pscan)
{
    for(int i = 1; i < clusterSupport.size(); i++) {
        const LineSegmentTuple &prev = clusterSupport[i - 1];
        const LineSegmentTuple &curr = clusterSupport[i];

        /* Join consecutive segments if the edges are close and if they are almost collinear */
        if(prev.segment.p2().distance2(curr.segment.p1())
                < SQUARE(Config::SLAM::mergeThreshold) &&

                std::abs(prev.segment.angle() - curr.segment.angle())
                < Config::SLAM::collinearityThreshold) {

            /* Fit line segment */
            const IndexSet joined = prev.index.join(curr.index);
            const UncertainLineSegment usegment = fittingFunction(pscan, joined);

            /* Find maximum distance from the line */
            double maxDist = findMaximumDistance(pscan, joined, usegment, NULL);

            /* Merge segments only if the maximum distance is acceptable */
            if(maxDist <= square(Config::SLAM::splitThreshold)) {
                const LineSegmentTuple tuple(usegment, joined);
                clusterSupport[i - 1] = tuple;
                clusterSupport.removeAt(i);
                i--;
            }
        }
    }

    for(int i = 0; i < clusterSupport.size(); i++) {
        LineSegmentTuple &tuple = clusterSupport[i];
        /* Treat segments consisting of < SM_MINIMUM_CLUSTER_SIZE points as noise */
        if(tuple.index.size() >= SM_MINIMUM_CLUSTER_SIZE) {
            segments.append(tuple.segment);
        } else {
            clusterSupport.removeAt(i);
            i--;
        }
    }

    for(int i = 1; i < segments.size(); i++) {
        UncertainLineSegment &prev = segments[i - 1];
        UncertainLineSegment &next = segments[i];

        /* If two consecutive segment endpoints are very close change the enpoints to the
           intersection of the two lines */
        if(prev.p2().distance(next.p1()) <= Config::SLAM::mergeThreshold) {
            Point intersection = prev.intersection(next);

            if(intersection.distance(prev.p2()) <= Config::SLAM::mergeThreshold &&
                    intersection.distance(next.p1()) <= Config::SLAM::mergeThreshold) {
                prev.setPsi2Projection(intersection);
                next.setPsi1Projection(intersection);
            }
        }

    }

    support << clusterSupport;
    clusterSupport.clear();
}

void SegmentScan::findLinesRANSAC(const PointScan &ps) {
    QList<Support::IndexSet> clusters = findClustersAggressive(ps);
    fforeach(const Support::IndexSet is, clusters) {
        findLinesSingleClusterRANSAC(ps, is);
    }
    qSort(support);
    fforeach(const LineSegmentTuple &s, support) {
        segments << s.segment;
    }
}

void SegmentScan::findLinesSingleClusterRANSAC(
        const PointScan &ps, const Support::IndexSet &cluster) {
    const int minimumPointCount = 5;
    QList<int> selectionPool = cluster.toList();
    Support::TopValues<1, QList<int> > best;
    best.add(0, QList<int>());
    while(best.count() > 0) {
        const int ransacIterations = min(3 * selectionPool.size(), 40);
        const int maxIndex = selectionPool.size() - 1;

        best.clear();
        for(int i = 0; i < ransacIterations; i++) {
            const Point &p1 = ps[selectionPool[Shared::Random::integer(0, maxIndex)]];
            const Point &p2 = ps[selectionPool[Shared::Random::integer(0, maxIndex)]];
            const LineSegment s(p1, p2);
            QList<int> *selected = new QList<int>, *current = new QList<int>;
            Point previous(INFINITY, INFINITY);

            fforeach(int j, selectionPool) {
                const Point &p = ps[j];
                if(s.distance(p) < Config::SLAM::splitThreshold) {
                    if(p.distance(previous) >= SM_CLUSTER_DISTANCE_THRESHOLD) {
                        if(current->size() > selected->size()) {
                            delete selected;
                            selected = current;
                        } else {
                            delete current;
                        }
                        current = new QList<int>;
                    }
                    current->append(j);
                    previous = p;
                }
            }

            if(current->size() > selected->size()) {
                delete selected;
                selected = current;
            } else {
                delete current;
            }

            if(selected->size() >= minimumPointCount)
                best.add(-selected->size(), *selected);

            delete selected;
        }

        if(best.count() > 0) {
            const QList<int> &points = best.value();
            Support::IndexSet is(points);
            support << LineSegmentTuple(totalLeastSquaresLineFit(ps, is), is);
            for(int i = 0, j = 0; j < points.size(); i++) {
                if(selectionPool[i] == points[j]) {
                    selectionPool.removeAt(i--);
                    j++;
                }
            }
        }
    }
}

static inline double pointRho(const Point &p, const Point &center) {
    return (p - center).norm();
}

inline std::pair<int, bool> SegmentScan::sweepNext(const QList<int> &open, int i) const {
    bool fromOpenSet = true;
    int index = -1, sweepIndex = INT_MAX;
    fforeach(int l, open) {
        if(support[l].endIndex() < sweepIndex) {
            index = l;
            sweepIndex = support[l].endIndex();
        }
    }
    sweepIndex--;
    if(i < support.length()) {
        if(support[i].startIndex() < sweepIndex) {
            index = i;
            sweepIndex = support[i].startIndex();
            fromOpenSet = false;
        }
    }
    return std::make_pair(index, fromOpenSet);
}

inline Point SegmentScan::findMaxIntersection(
        const QList<int> &open, const LineSegment &jump) const {
    Point maxIntersection;
    double maxRho = -1;

    fforeach(int l, open) {
        Point intersection = jump.intersection(support[l].segment);
        double rho = pointRho(intersection, center);
        if(rho > maxRho) {
            maxIntersection = intersection;
            maxRho = rho;
        }
    }

    return maxIntersection;
}

void SegmentScan::findFrontiersAndPolygon(const PointScan &pscan) {
    center = pscan.center();

    int i = 0;
    QList<int> open;
    Point previous = center;
    while(i < support.size() || open.size() > 0) {
        if(open.size() == 0) {
            open.append(i);
            frontiers.append(Frontier(previous, support[i].segment.p1()));
            polygon.edges.append(frontiers.last());
            //vertices.append(support[i].segment.p1());
            previous = support[i].segment.p1();
            i++;
        } else {
            std::pair<int, bool> next = sweepNext(open, i);

            if(next.second) {
                open.removeOne(next.first);
                const Point &end = support[next.first].segment.p2();
                if(open.empty()) {
                    polygon.edges.append(LineSegment(previous, end));
                    previous = end;
                    //vertices.append(end);
                } else {
                    LineSegment jump(center, end);
                    Point maxIntersection = findMaxIntersection(open, jump);

                    if(pointRho(maxIntersection, center) < pointRho(end, center)) {
                        frontiers.append(Frontier(end, maxIntersection));
                        polygon.edges.append(LineSegment(previous, end));
                        polygon.edges.append(frontiers.last());
                        previous = maxIntersection;
                        //vertices.append(end);
                        //vertices.append(maxIntersection);
                    }
                }
            } else {
                const Point &start = support[next.first].segment.p1();
                LineSegment jump(center, start);
                Point maxIntersection = findMaxIntersection(open, jump);

                if(pointRho(maxIntersection, center) < pointRho(start, center)) {
                    frontiers.append(Frontier(maxIntersection, start));
                    polygon.edges.append(LineSegment(previous, maxIntersection));
                    polygon.edges.append(frontiers.last());
                    previous = start;
                    //vertices.append(maxIntersection);
                    //vertices.append(start);
                }
                open.append(next.first);
                i++;
            }
        }
    }

    frontiers.append(Frontier(previous, center));
    polygon.edges.append(frontiers.last());

    if(frontiers.size() >= 2 &&
            frontiers.first().p2().almostEqual(frontiers.last().p1(),
                    Config::SLAM::mergeThreshold)) {
        frontiers.removeLast();
        frontiers.removeFirst();
        Point prev = polygon.edges.last().p1();
        Point next = polygon.edges.first().p2();
        polygon.edges.removeFirst();
        polygon.edges.last() = LineSegment(prev, next);
    }

    for(int i = 0; i < frontiers.size(); i++) {
        if(frontiers[i].length() < Config::SLAM::mergeThreshold) {
            frontiers.removeAt(i--);
        }
    }

    support.clear();
}

const VisibilityPolygon &SegmentScan::toPolygon() const {
    return polygon;
}

SegmentScan operator*(const Geometry::Rototranslation &rt, const SegmentScan &scan)
{
    SegmentScan rototranslated;
    fforeach(const UncertainLineSegment &l, scan.segments) {
        rototranslated.segments.append(rt * l);
    }
    fforeach(const Frontier &f, scan.frontiers) {
        rototranslated.frontiers.append(rt * f);
    }
    rototranslated.center = rt * scan.center;
    rototranslated.polygon = rt * scan.polygon;
    return rototranslated;
}

} /* namespace Geometry */
} /* namespace SLAM */
