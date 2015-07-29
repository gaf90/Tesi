/*
 * uninformedassociationbase.h
 *
 *  Created on: 25/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef UNINFORMEDASSOCIATIONBASE_H_
#define UNINFORMEDASSOCIATIONBASE_H_

#include "associationbase.h"

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class UninformedAssociationBase : public AssociationBase<Derived>
{
public:
    template <typename S>
    static double distance(Retriever<S> &retr, int queryidx, int mapidx);
};

template <typename Derived> template <typename S>
inline double UninformedAssociationBase<Derived>::distance(
        Retriever<S> &retr, int queryidx, int mapidx)
{
    return Derived::distance(retr.querySegment(queryidx), retr.mapSegment(mapidx));
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#endif /* UNINFORMEDASSOCIATIONBASE_H_ */
