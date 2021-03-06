/*
 * associationelseberg.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef ASSOCIATIONELSEBERG_H_
#define ASSOCIATIONELSEBERG_H_

#include "uninformedassociationbase.h"
#include "nonoverlapdistance.h"
#include "slam/utilities.h"

namespace SLAM {
namespace ScanMatching {

/*
 * Elseberg, J., Creed, R., Lakaemper, R.: A line segment based system for 2D global mapping.
 * In: Proc. ICRA. (2010) 3924–3931
 */
class AssociationElseberg : public UninformedAssociationBase<AssociationElseberg>
{
public:
    using UninformedAssociationBase<AssociationElseberg>::distance;
    static double distance(const Geometry::LineSegment &s1, const Geometry::LineSegment &s2);

    static const double &lookupThreshold;
    static const double &broadLookupThreshold;
};

inline double AssociationElseberg::distance(
        const Geometry::LineSegment &s1, const Geometry::LineSegment &s2)
{
	using namespace Geometry;

	double dang, duva, duv;
	double p1s1, p2s1, p1s2, p2s2;
    double was1 = s1.angle(), was2 = s2.angle();
    double ang_a = weightedLineAngle(was1, was2, 1, 1);
	Point cen_a = .5 * (s1.centroid() + s2.centroid());

	/*
    dang = min(s1.length(), 2.) * std::abs(std::tan(linewrap(was1 - ang_a))) +
            min(s2.length(), 2.) * std::abs(std::tan(linewrap(was2 - ang_a)));
    */
	dang = s1.length() * std::abs(std::tan(linewrap(was1 - ang_a))) +
	        s2.length() * std::abs(std::tan(linewrap(was2 - ang_a)));

	double line_a = std::sin(ang_a), line_b = -std::cos(ang_a);
	double line_c = -cen_a.x() * line_a - cen_a.y() * line_b;

	p1s1 = s1.p1().distance2(LineSegment::project(line_a, line_b, line_c, s1.p1()));
	p2s1 = s1.p2().distance2(LineSegment::project(line_a, line_b, line_c, s1.p2()));
	p1s2 = s2.p1().distance2(LineSegment::project(line_a, line_b, line_c, s2.p1()));
	p2s2 = s2.p2().distance2(LineSegment::project(line_a, line_b, line_c, s2.p2()));

	duva = std::sqrt(max(p1s1, p2s1, p1s2, p2s2)); // Questo è un errore

	p1s1 = s1.p1().distance2(s2.project(s1.p1()));
	p2s1 = s1.p2().distance2(s2.project(s1.p2()));
	p1s2 = s2.p1().distance2(s1.project(s2.p1()));
	p2s2 = s2.p2().distance2(s1.project(s2.p2()));

	duv = std::sqrt(min(p1s1, p2s1, p1s2, p2s2));

    return 0.66 * dang + 0.09 * duva + 0.25 * duv + nonOverlapDistance(s1, s2);
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* ASSOCIATIONELSEBERG_H_ */
