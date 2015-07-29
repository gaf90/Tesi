/*
 * semideterministicretriever.h
 *
 *  Created on: 05/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef SEMIDETERMINISTICRETRIEVER_H_
#define SEMIDETERMINISTICRETRIEVER_H_

#include "slam/scanmatching/informedretriever.h"
#include "slam/geometry/linesegment.h"
#include "slam/geometry/segmentscan.h"
#include "slam/support/alignedvector.h"
#include <QList>

#ifdef EVALUATION_TESTING
#   include "slam/support/stopwatch.h"
#endif

namespace SLAM {
namespace Engine {

/*
    Practically equal to DeterministicRetriever plus pose covariance information. The code needs
    to be duplicated in order to avoid diamond inheritance.
*/
class SemiDeterministicRetriever :
        public ScanMatching::InformedRetriever<SemiDeterministicRetriever>
{
public:
    inline SemiDeterministicRetriever(
            const QList<Geometry::LineSegment *> &map, const Geometry::SegmentScan &scan,
            const Geometry::UncertainRototranslation &pose) :
            map(&map), query(scan), rotatedQuery(scan.getSegments().size()),
            transformedPose(pose), initialPose(pose)
    {
        applyTransformation();
    }

    inline SemiDeterministicRetriever(const SemiDeterministicRetriever &sdr) :
            map(sdr.map), query(sdr.query), rotatedQuery(sdr.rotatedQuery),
            transformedPose(sdr.transformedPose), initialPose(sdr.initialPose)
    {
    }

    inline SemiDeterministicRetriever &operator=(const SemiDeterministicRetriever &sdr)
    {
        map = sdr.map;
        query = sdr.query;
        rotatedQuery = sdr.rotatedQuery;
        transformedPose = sdr.transformedPose;
        initialPose = sdr.initialPose;
        return *this;
    }

    inline const Geometry::SegmentScan &scan() const {
        return query;
    }

    inline int mapSegmentCount() const {
        return map->size();
    }

    inline int querySegmentCount() const {
        return rotatedQuery.size();
    }

    inline const Geometry::LineSegment &mapSegment(int index) {
        return *map->at(index);
    }

    inline const Geometry::LineSegment &querySegment(int index) {
        return rotatedQuery[index];
    }

    inline const Geometry::Rototranslation &queryPose() {
        return transformedPose;
    }

    inline const Geometry::Rototranslation &initialQueryPose() {
        return initialPose;
    }

    inline void applyTransformation(const Geometry::Rototranslation &rt) {
        transformedPose = rt * transformedPose;
        applyTransformation();
    }

    inline const Eigen::Matrix3d &queryPoseCovariance() {
        return initialPose.covariance();
    }

    inline const Eigen::Matrix4d &querySegmentCovariance(int index) {
        return rotatedQuery[index].covariance();
    }

    inline const Eigen::Matrix4d &querySegmentScanCovariance(int index) {
        return query.getSegments()[index].covariance();
    }

private:
    inline void applyTransformation() {
        for(int i = 0; i < querySegmentCount(); i++) {
            const Geometry::UncertainLineSegment &s = query.getSegments()[i];
            rotatedQuery[i] = transformedPose * s;
            //ldbg << rotatedQuery[i] << endl <<
            //        "\t" << rotatedQuery[i].covariance() << endl <<
            //        "\t" << query.getSegments()[i].covariance() << endl;
        }
    }


private:
    const QList<Geometry::LineSegment *> *map;
    Geometry::SegmentScan query;
    AlignedVector<Geometry::UncertainLineSegment> rotatedQuery;
    Geometry::UncertainRototranslation transformedPose, initialPose;

#ifdef EVALUATION_TESTING
public:
    Support::Stopwatch stopwatch;
#endif
};

} /* namespace Engine */
} /* namespace SLAM */

#endif /* SEMIDETERMINISTICRETRIEVER_H_ */
