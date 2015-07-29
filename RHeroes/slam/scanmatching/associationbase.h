/*
 * associationbase.h
 *
 *  Created on: 16/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef ASSOCIATIONBASE_H_
#define ASSOCIATIONBASE_H_

#include <QList>
#include "slam/geometry/segmentscan.h"
#include "slam/support/topvalues.h"
#include "slam/utilities.h"
#include "shared/config.h"
#include "retriever.h"
#ifdef EVALUATION_TESTING
#   include "slam/engine/semideterministicretriever.h"
#endif /* EVALUATION_TESTING */

namespace SLAM {
namespace ScanMatching {

template <typename Derived>
class AssociationBase
{
public:
    template <typename S>
    static double distance(Retriever<S> &retr, int queryidx, int mapidx);

    template <int N, typename S>
    static int lookup(
            double distanceCutoff, Retriever<S> &retr,
            QList<int> associations[]);

    template <int N, typename S>
    static int lookup(Retriever<S> &retr, QList<int> associations[]);

    template <typename S>
    static int broadLookup(Retriever<S> &retr, QList<int> associations[]);
};

template <typename Derived> template <typename S>
inline double AssociationBase<Derived>::distance(Retriever<S> &retr, int queryidx, int mapidx)
{
    return Derived::distance(retr.derived(), queryidx, mapidx);
}

#ifdef EVALUATION_TESTING
namespace __internal {
inline void stopwatchStart(Engine::SemiDeterministicRetriever &sdr) {
    sdr.stopwatch.start();
}

inline void stopwatchStop(Engine::SemiDeterministicRetriever &sdr) {
    sdr.stopwatch.stop();
}

template <typename Dummy>
inline void stopwatchStart(Dummy &r) {
    Q_UNUSED(r)
}

template <typename Dummy>
inline void stopwatchStop(Dummy &r) {
    Q_UNUSED(r)
}
} /* namespace __internal */
#endif /* EVALUATION_TESTING */

template <typename Derived> template <int N, typename S>
inline int AssociationBase<Derived>::lookup(
        double distanceCutoff, Retriever<S> &retr, QList<int> associations[])
{
#ifdef EVALUATION_TESTING
    __internal::stopwatchStart(retr.derived());
#endif /* EVALUATION_TESTING */
    int count = 0;
    const int queryCount = retr.querySegmentCount(), mapCount = retr.mapSegmentCount();

    for(int i = 0; i < queryCount; i++) {
        Support::TopValues<N> tv;
        associations[i].clear();
        for(int j = 0; j < mapCount; j++) {
        	double d = Derived::distance(retr.derived(), i, j);
            //ldbg << "d(" << i << "," << j << ") = " << d << endl;
        	if(d <= distanceCutoff) {
        	    tv.add(d, j);
        	    //if(i == 4) ldbg << "d(" << i << "," << j << ") = " << d << endl;
        	}
        }
        for(int j = 0; j < tv.count(); j++) {
            associations[i].append(tv.value(j));
            count++;
        }
    }

#ifdef EVALUATION_TESTING
    __internal::stopwatchStop(retr.derived());
#endif /* EVALUATION_TESTING */

    return count;
}

template <typename Derived> template <int N, typename S>
inline int AssociationBase<Derived>::lookup(Retriever<S> &retr, QList<int> associations[])
{
    return AssociationBase<Derived>::template lookup<N, S>(
            Derived::lookupThreshold, retr, associations);
}

template <typename Derived> template <typename S>
inline int AssociationBase<Derived>::broadLookup(Retriever<S> &retr, QList<int> associations[])
{
    return AssociationBase<Derived>::template lookup<8, S>(
            Derived::broadLookupThreshold, retr, associations);
}

} /* namespace ScanMatching */
} /* namespace SLAM */

#include "associationposecentric.h"

#endif /* ASSOCIATIONBASE_H_ */
