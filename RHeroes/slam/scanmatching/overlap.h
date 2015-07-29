/*
 * overlap.h
 *
 *  Created on: 31/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef OVERLAP_H_
#define OVERLAP_H_

#include "retriever.h"
#include <QList>
#include <QtAlgorithms>

namespace SLAM {
namespace ScanMatching {

namespace __internal {

typedef std::pair<double, double> Interval1D;

static inline bool lessThanBegin(const Interval1D &p, double begin) {
    return p.second < begin;
}

static inline bool lessThanEnd(const Interval1D &p, double end) {
    return p.first < end;
}

class Set1D {
public:
    inline void add(double begin, double end) {
        const QList<std::pair<double, double> >::const_iterator ibegin = intervals.begin(),
                iend = intervals.end();
        int after  = qLowerBound(ibegin, iend, begin, lessThanBegin) - ibegin;
        int before = qLowerBound(ibegin, iend, end,   lessThanEnd)   - ibegin;
        double newbegin, newend;
        if(after < intervals.size() && intervals[after].first < begin) {
            newbegin = intervals[after].first;
        } else {
            newbegin = begin;
        }

        if(before > 0 && intervals[before - 1].second > end) {
            newend = intervals[before - 1].second;
        } else {
            newend = end;
        }

        for(int i = after; i < before; i++) {
            intervals.removeAt(after);
        }

        intervals.insert(after, std::make_pair(newbegin, newend));
    }

    inline double overlapAmount(double begin, double end) const {
        double sum = 0;
        fforeach(const Interval1D &p, intervals) {
            sum += max(0., min(end, p.second) - max(begin, p.first));
        }
        return sum;
    }

private:
    QList<Interval1D> intervals;
};

} /* namespace __internal */

template <typename S>
inline double overlapAmount(
        Retriever<S> &retriever, const Geometry::Rototranslation &rt, int scanidx, int refidx)
{
    const Geometry::LineSegment s = rt * retriever.querySegment(scanidx);
    const Geometry::LineSegment &r = retriever.mapSegment(refidx);
    const Geometry::LineSegment merged = s.merge(r);
    const double p11 = merged.project1D(s.p1()), p12 = merged.project1D(s.p2());
    const double p21 = merged.project1D(r.p1()), p22 = merged.project1D(r.p2());

    return max(0., min(max(p11, p12), max(p21, p22)) - max(min(p11, p12), min(p21, p22)));
}

template <typename S>
inline double overlapAmount(
        Retriever<S> &retriever, const Geometry::Rototranslation &rt,
        int scanidx, const QList<int> &refassoc)
{
    int count = refassoc.size();
    if(count > 0) {
        /* Find merge line by tree descent merge */
        QList<Geometry::LineSegment> pool;
        fforeach(int i, refassoc) {
            pool.append(retriever.mapSegment(i));
        }

        while(count > 1) {
            if(count % 2 == 1) {
                pool.append(pool.first());
                pool.pop_front();
            }
            for(int i = 0; i < count / 2; i++) {
                pool[i].mergeInPlace(pool[i + 1]);
                pool.removeAt(i + 1);
            }
            count = pool.size();
        }

        const Geometry::LineSegment &merged = pool.first();

        /* Find overlap amount of intervals over 1D projections */
        __internal::Set1D intervals;
        fforeach(int i, refassoc) {
            const Geometry::LineSegment &r = retriever.mapSegment(i);
            const double r1 = merged.project1D(r.p1()), r2 = merged.project1D(r.p2());
            if(r1 < r2) {
                intervals.add(r1, r2);
            } else {
                intervals.add(r2, r1);
            }
        }

        const Geometry::LineSegment s = rt * retriever.querySegment(scanidx);
        const double s1 = merged.project1D(s.p1()), s2 = merged.project1D(s.p2());

        if(s1 < s2) {
            return intervals.overlapAmount(s1, s2);
        } else {
            return intervals.overlapAmount(s2, s1);
        }
    } else {
        return 0;
    }
}

template <typename S>
inline double overlapAmount(
        Retriever<S> &retriever, const Geometry::Rototranslation &rt,
        const QList<int> *associations)
{
    double sum = 0;
    const int querySegmentCount = retriever.querySegmentCount();
    for(int i = 0; i < querySegmentCount; i++) {
        const int count = associations[i].size();
        if(count > 1) {
            sum += overlapAmount(retriever, rt, i, associations[i]);
        } else if(count == 1) {
            sum += overlapAmount(retriever, rt, i, associations[i].first());
        }
    }
    return sum;
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* OVERLAP_H_ */
