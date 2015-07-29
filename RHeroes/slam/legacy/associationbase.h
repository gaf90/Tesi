/*
 * associationbase.h
 *
 *  Created on: 16/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef LEGACY_ASSOCIATIONBASE_H_
#define LEGACY_ASSOCIATIONBASE_H_

#include <QList>
#include "slam/geometry/segmentscan.h"
#include "slam/support/topvalues.h"
#include "slam/utilities.h"
#include "shared/config.h"

namespace SLAM {
namespace Legacy {

template <typename T>
class AssociationBase
{
public:

    template <typename S>
    static double distance(const S &s1, const S &s2);

    template <int N>
    static int lookup(
            const Geometry::SegmentScan &s,
            const QList<Geometry::LineSegment *> &map,
            QList<int> associations[]);
    template <int N>
    static int lookup(
            const Geometry::SegmentScan &s,
            const QList<Geometry::UncertainLineSegment *> &map,
            QList<int> associations[]);

private:
    template <typename S, int N>
    static int lookup(
            double distanceCutoff,
            const Geometry::SegmentScan &s,
            const QList<S *> &map,
            QList<int> associations[]);
};


template <typename T> template <typename S>
inline double AssociationBase<T>::distance(const S &s1, const S &s2)
{
    return T::distance(s1, s2);
}

template <typename T> template <typename S, int N>
inline int AssociationBase<T>::lookup(
        double distanceCutoff,
        const Geometry::SegmentScan &s,
        const QList<S *> &map,
        QList<int> associations[])
{
    int i = 0, count = 0;
    const AlignedVector<Geometry::UncertainLineSegment> &segments = s.getSegments();

    fforeach(const Geometry::UncertainLineSegment &l, segments) {
        int j = 0;
        Support::TopValues<N> tv;
        associations[i].clear();
        fforeach(const S *f, map) {
        	double d = T::distance(l, *f);
        	if(d <= distanceCutoff) tv.add(d, j);
        	j++;
        }
        for(j = 0; j < tv.count(); j++) {
            associations[i].append(tv.value(j));
            count++;
        }
        i++;
    }

#if 0
    double dmean = 0;
    for(i = 0; i < segments.size(); i++) {
        fforeach(int j, associations[i]) {
            dmean += T::distance(segments[i], *map[j]);
        }
    }
    dmean /= count;

    for(i = 0; i < segments.size(); i++) {
        for(int j = 0; j < associations[i].size(); j++) {
            if(std::abs(T::distance(segments[i], *map[associations[i][j]]) - dmean) > 0.3) {
                associations[i].removeAt(j);
                j--;
            }
        }
    }
#endif
    return count;
}

template <typename T> template <int N>
inline int AssociationBase<T>::lookup(
        const Geometry::SegmentScan &s,
        const QList<Geometry::LineSegment *> &map,
        QList<int> associations[])
{
    return AssociationBase<T>::template lookup<Geometry::LineSegment, N>(
            Config::SLAM::lookupThresholdElseberg, s, map, associations);
}

template <typename T> template <int N>
inline int AssociationBase<T>::lookup(
        const Geometry::SegmentScan &s,
        const QList<Geometry::UncertainLineSegment *> &map,
        QList<int> associations[])
{
    return AssociationBase<T>::template lookup<Geometry::UncertainLineSegment, N>(
            Config::SLAM::lookupThresholdElseberg, s, map, associations);
}

} /* namespace Legacy */
} /* namespace SLAM */

#endif /* ASSOCIATIONBASE_H_ */
