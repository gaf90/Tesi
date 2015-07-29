/*
 * retriever.h
 *
 *  Created on: 25/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef RETRIEVER_H_
#define RETRIEVER_H_

#include "slam/geometry/rototranslation.h"
#include "slam/geometry/linesegment.h"

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class Retriever
{
public:
    typedef Derived DerivedType;

    inline Derived &derived() {
        return static_cast<Derived &>(*this);
    }

    inline const Derived &derived() const {
        return static_cast<const Derived &>(*this);
    }

    inline int mapSegmentCount() const {
        return derived().mapSegmentCount();
    }

    inline int querySegmentCount() const {
        return derived().querySegmentCount();
    }

    inline const Geometry::LineSegment mapSegment(int index) {
        return derived().mapSegment(index);
    }

    inline const Geometry::LineSegment querySegment(int index) {
        return derived().querySegment(index);
    }

    inline const Geometry::Rototranslation queryPose() {
        return derived().queryPose();
    }

    inline const Geometry::Rototranslation initialQueryPose() {
        return derived().initialQueryPose();
    }

    inline void applyTransformation(const Geometry::Rototranslation &rt) {
        derived().applyTransformation(rt);
    }

    inline void resetTransformation() {
        applyTransformation(initialQueryPose() * queryPose().inverse());
    }

};

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* ASSOCIATIONAMIGONI_H_ */
