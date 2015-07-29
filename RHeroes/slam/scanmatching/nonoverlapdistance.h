/*
 * nonoverlapdistance.h
 *
 *  Created on: 20/mag/2012
 *      Author: Mladen Mazuran
 */

#ifndef NONOVERLAPDISTANCE_H_
#define NONOVERLAPDISTANCE_H_

#include "slam/geometry/linesegment.h"

namespace SLAM {
namespace ScanMatching {

#if 0
static inline double oneWayNonOverlapDistance(
        const Geometry::LineSegment &s1, const Geometry::LineSegment &s2)
{
    const double l = s1.length();
    double p1 = s1.project1D(s2.p1()), p2 = s1.project1D(s2.p2());

    /* Orient the projections consistently with segment s1 */
    if(p1 > p2) exchange(p1, p2);

    if(p1 < 0) {
        if(p2 > l) {
            /* s2 fully covers s1 */
            return 0;
        } else {
            /* It doesn't matter whether p2 is in [0,l] or not. The formula is the same */
            return l - p2;
        }
    } else if(p1 <= l) {
        if(p2 > l) {
            /* s1 is not covered by s2 only up to p1 */
            return p1;
        } else {
            /* s1 is covered by s2 only between p1 and p2 */
            return p1 + l - p2;
        }
    } else {
        /* if p1 > l then also p2 > l */
        return p1;
    }
}
#endif

static inline double nonOverlapDistance(
        const Geometry::LineSegment &s1, const Geometry::LineSegment &s2)
{
    const Geometry::LineSegment merged = s1.merge(s2);
    const double p11 = merged.project1D(s1.p1()), p12 = merged.project1D(s1.p2());
    const double p21 = merged.project1D(s2.p1()), p22 = merged.project1D(s2.p2());

    return max(0., max(min(p11, p12) - max(p21, p22), min(p21, p22) - max(p11, p12)));

    //return min(oneWayNonOverlapDistance(s1, s2), oneWayNonOverlapDistance(s2, s1));
}

} /* namespace ScanMatching */
} /* namespace SLAM */


#endif /* NONOVERLAPDISTANCE_H_ */
