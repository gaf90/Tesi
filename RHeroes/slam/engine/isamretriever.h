/*
 * isamretriever.h
 *
 *  Created on: 04/feb/2013
 *      Author: Mladen Mazuran
 */

#ifndef ISAMRETRIEVER_H_
#define ISAMRETRIEVER_H_

#include "isamlandmark.h"
#include "slam/geometry/segmentscan.h"
#include "slam/scanmatching/fullyinformedretriever.h"
#include <isam/Slam.h>
#include <QtAlgorithms>

namespace SLAM {
namespace Engine {

class ISAMRetriever : public ScanMatching::FullyInformedRetriever<ISAMRetriever>
{
public:
    ISAMRetriever(
            ISAMLandmark *robot, ISAMLandmark *landmark,
            const Eigen::Matrix3d &Srr, const Eigen::Matrix3d &Srl,
            const Eigen::Matrix3d &Sll, const Geometry::SegmentScan &query) :
        robot(robot), landmark(landmark), Srl(Srl), Sll(Sll),
        transformedPose(robot->pose(), Srr), query(query)
    {
    }


    inline void applyTransformation(const Geometry::Rototranslation &rt) {
        transformedPose = rt * transformedPose;
        Eigen::Matrix3d R;
        R <<
                rt.cosAngle(), -rt.sinAngle(), 0,
                rt.sinAngle(),  rt.cosAngle(), 0,
                            0,              0, 1;
        Srl = (R * Srl).eval();
    }

    inline ISAMLandmark *associatedLandmark(int mapidx) {
        Q_UNUSED(mapidx)
        return landmark;
    }

    inline const Geometry::SegmentScan &scan() const {
        return query;
    }


    /* Counting functions */

    inline int querySegmentCount() const {
        return query.getSegments().size();
    }

    inline int mapSegmentCount() const {
        return landmark->propagatedSegments().size();
    }


    /* Line segment retrieval functions */

    inline const Geometry::LineSegment querySegment(int index) {
        return static_cast<Geometry::Rototranslation &>(transformedPose) *
                static_cast<const Geometry::LineSegment &>(query.getSegments()[index]);
    }

    inline const Geometry::LineSegment &mapSegment(int index) {
        return landmark->propagatedSegments()[index];
    }


    /* Line segment covariance retrieval functions */

    inline const Eigen::Matrix4d querySegmentCovariance(int index) {
        return (transformedPose * query.getSegments()[index]).covariance();
    }

    inline const Eigen::Matrix4d mapSegmentCovariance(int index) {
        return (Geometry::UncertainRototranslation(landmark->pose(), Sll) *
                landmark->segments()[index]).covariance();
    }

    inline const Eigen::Matrix4d queryMapSegmentsJointCovariance(int queryidx, int mapidx) {
        return Geometry::UncertainLineSegment::transformationJacobianRT(
                        query.getSegments()[queryidx].alpha(), transformedPose) *
                Srl * Geometry::UncertainLineSegment::transformationJacobianRT(
                        landmark->segments()[mapidx].alpha(), landmark->pose()).transpose();
    }

    inline const Eigen::Matrix4d querySegmentScanCovariance(int index) {
        return (static_cast<Geometry::Rototranslation &>(transformedPose) *
                query.getSegments()[index]).covariance();
    }

    inline const Eigen::Matrix4d mapSegmentScanCovariance(int index) {
        return landmark->propagatedSegments()[index].covariance();
    }


    /* Pose retrieval functions */

    inline const Geometry::Rototranslation initialQueryPose() {
        return robot->pose();
    }

    inline const Geometry::Rototranslation &queryPose() {
        return transformedPose;
    }

    inline Geometry::Rototranslation mapPose(int index) {
        Q_UNUSED(index)
        return landmark->pose();
    }


    /* Pose covariance retrieval functions */

    inline const Eigen::Matrix3d &queryPoseCovariance() const {
        return transformedPose.covariance();
    }

    inline const Eigen::Matrix3d &mapPoseCovariance(int index) const {
        Q_UNUSED(index)
        return Sll;
    }

    inline const Eigen::Matrix3d &queryPoseMapPoseJointCovariance(int mapidx) const {
        Q_UNUSED(mapidx)
        return Srl;
    }

private:
    ISAMLandmark *robot, *landmark;
    Eigen::Matrix3d Srl;
    const Eigen::Matrix3d &Sll;
    Geometry::UncertainRototranslation transformedPose;
    Geometry::SegmentScan query;
};

} /* namespace Engine */
} /* namespace SLAM */
#endif /* ISAMRETRIEVER_H_ */
