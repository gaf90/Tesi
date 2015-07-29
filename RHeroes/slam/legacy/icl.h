/*
 * icl.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef LEGACY_ICL_H_
#define LEGACY_ICL_H_

#include "slam/constants.h"
#include "slam/utilities.h"
#include "slam/geometry/rototranslation.h"
#include "slam/geometry/segmentscan.h"
#include <QList>
#include <Eigen/Core>

namespace SLAM {
namespace Legacy {

template <typename A, typename M, typename T>
class ICL {
public:
	ICL(
            const Geometry::SegmentScan &query,
			const QList<T *> &map,
			const Geometry::Rototranslation &guess);
	virtual ~ICL();

	bool run();
	inline const Eigen::Vector3d &measure() const { return rt; }
	inline const QList<int> *associations() const { return assoc; }
	inline QList<int> *associations(bool yield = false) { owner = !yield; return assoc; }

private:
    const Geometry::SegmentScan &query;
	const QList<T *> &map;
	Eigen::Vector3d rt;
	QList<int> *assoc;
	bool owner;
};

template <typename A, typename M, typename T>
inline ICL<A, M, T>::ICL(
        const Geometry::SegmentScan &query,
		const QList<T *> &map,
		const Geometry::Rototranslation &guess) :
	query(query), map(map), rt(guess), owner(true)
{
	assoc = new QList<int>[query.getSegments().size()];
}

template <typename A, typename M, typename T>
inline ICL<A, M, T>::~ICL()
{
    if(owner)
        delete[] assoc;
}

template <typename A, typename M, typename T>
bool ICL<A, M, T>::run()
{
	int counter = 0;
	Eigen::Vector3d prev(0, 0, 0), curr(rt);

	do {
	    Geometry::Rototranslation stepRT = Geometry::Rototranslation(curr) *
	            Geometry::Rototranslation(prev);
        Geometry::SegmentScan rotoQuery = stepRT * query;
	    prev = stepRT;
		bool foundSomething = A::template lookup<1>(rotoQuery, map, assoc);
		if(!foundSomething) {
		    rt = Eigen::Vector3d::Zero();
            return false;
		}

		curr = M::minimize(rotoQuery, map, assoc);
#ifdef VERBOSE_DEBUG
        ldbg << "Graphics[{" << query.getSegments() << "," << map << "}]" << endl;
        ldbg << arrayprint(assoc, rotoQuery.getSegments().size()) << endl;
        ldbg << curr << endl;
        ldbg << prev << endl;
        ldbg << "Graphics[{" << (Geometry::Rototranslation(curr) * rotoQuery).getSegments() << "," << map << "}]" << endl << endl;
#endif /* VERBOSE_DEBUG */
		counter++;
	} while((SQUARE(curr[0]) + SQUARE(curr[1]) > SQUARE(SM_ICL_CONVERGENCE_THRESHOLD) ||
	        std::abs(wrap(curr[2])) > SM_ICL_CONVERGENCE_THRESHOLD) &&
	        counter < SM_ICL_MAXIMUM_ITERATIONS);

	rt = Geometry::Rototranslation(curr) * Geometry::Rototranslation(prev);
	return true;
}

} /* namespace Legacy */
} /* namespace SLAM */
#endif /* LEGACY_ICL_H_ */
