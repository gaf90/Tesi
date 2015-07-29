/*
 * associationamigoni.h
 *
 *  Created on: 20/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef LEGACY_ASSOCIATIONAMIGONI_H_
#define LEGACY_ASSOCIATIONAMIGONI_H_

#include "associationbase.h"
#include "slam/scanmatching/nonoverlapdistance.h"

namespace SLAM {
namespace Legacy {

/*
 * Amigoni, F., Gasparini, S., Gini, M.: Building Segment-Based Maps Without Pose Information.
 * Proceedings of the IEEE 90(7) (2006) 1340-1359
 */
class AssociationAmigoni : public AssociationBase<AssociationAmigoni>
{
public:
    static double distance(const Geometry::LineSegment &s1, const Geometry::LineSegment &s2);
};

inline double AssociationAmigoni::distance(
        const Geometry::LineSegment &s1, const Geometry::LineSegment &s2)
{
	const double p1s1 = s2.distance2(s1.p1());
	const double p2s1 = s2.distance2(s1.p2());
	const double p1s2 = s1.distance2(s2.p1());
	const double p2s2 = s1.distance2(s2.p2());
	const double damigoni = std::sqrt(min(max(p1s1, p2s1), max(p1s2, p2s2)));

    return damigoni + ScanMatching::nonOverlapDistance(s1, s2);
}


} /* namespace Legacy */
} /* namespace SLAM */

#endif /* LEGACY_ASSOCIATIONAMIGONI_H_ */
