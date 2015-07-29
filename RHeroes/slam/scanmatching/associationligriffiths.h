/*
 * associationligriffiths.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef ASSOCIATIONLIGRIFFITHS_H_
#define ASSOCIATIONLIGRIFFITHS_H_

#include "uninformedassociationbase.h"

namespace SLAM {
namespace ScanMatching {

/*
 * Li, Q., Griffiths, J.: Iterative closest geometric objects registration.
 * Computers & Mathematics with Applications 40(10-11) (2000) 1171–1188
 */
class AssociationLiGriffiths : public UninformedAssociationBase<AssociationLiGriffiths>
{
public:
    using UninformedAssociationBase<AssociationLiGriffiths>::distance;
    static double distance(const Geometry::LineSegment &s1, const Geometry::LineSegment &s2);

    static const double &lookupThreshold;
    static const double &broadLookupThreshold;
};

inline double AssociationLiGriffiths::distance(
        const Geometry::LineSegment &s1, const Geometry::LineSegment &s2)
{
    double l1 = s1.length(), l2 = s2.length();
    return ((s1.p1() - s2.p1()).norm2() + (s1.p2() - s2.p2()).norm2() +
            (s1.p1() - s2.p1()) * (s1.p2() - s2.p2())) * (l1 + l2) / 6;
}


} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* ASSOCIATIONLIGRIFFITHS_H_ */
