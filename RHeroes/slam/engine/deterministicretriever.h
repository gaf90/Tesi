/*
 * deterministicretriever.h
 *
 *  Created on: 26/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef DETERMINISTICRETRIEVER_H_
#define DETERMINISTICRETRIEVER_H_

#include "slam/scanmatching/retriever.h"
#include "slam/geometry/linesegment.h"
#include "slam/geometry/segmentscan.h"
#include <QList>

namespace SLAM {
namespace Engine {

class DeterministicRetriever : public ScanMatching::Retriever<DeterministicRetriever>
{
public:
    inline DeterministicRetriever(
            const QList<Geometry::LineSegment *> &map, const Geometry::SegmentScan &scan,
            const Geometry::Rototranslation &pose) :
            map(&map), query(pose * scan), transformedPose(pose),
            initialPose(pose)
    {
    }

    inline DeterministicRetriever(const DeterministicRetriever &dr) :
            map(dr.map), query(dr.query), transformedPose(dr.transformedPose),
            initialPose(dr.initialPose)
    {
    }

    inline DeterministicRetriever &operator=(const DeterministicRetriever &dr)
    {
        map = dr.map;
        query = dr.query;
        transformedPose = dr.transformedPose;
        initialPose = dr.initialPose;
        return *this;
    }

    inline const Geometry::SegmentScan &scan() const {
        return query;
    }

    inline int mapSegmentCount() const {
        return map->size();
    }

    inline int querySegmentCount() const {
        return query.getSegments().size();
    }

    inline const Geometry::LineSegment &mapSegment(int index) {
        return *map->at(index);
    }

    inline const Geometry::LineSegment &querySegment(int index) {
        return query.getSegments()[index];
    }

    inline const Geometry::Rototranslation &queryPose() {
        return transformedPose;
    }

    inline const Geometry::Rototranslation &initialQueryPose() {
        return initialPose;
    }

    inline void applyTransformation(const Geometry::Rototranslation &rt) {
        transformedPose = rt * transformedPose;
        query = rt * query;
    }

private:
    const QList<Geometry::LineSegment *> *map;
    Geometry::SegmentScan query;
    Geometry::Rototranslation transformedPose, initialPose;
};

} /* namespace Engine */
} /* namespace SLAM */

#endif /* DETERMINISTICRETRIEVER_H_ */
