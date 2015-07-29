/*
 * isamslam.h
 *
 *  Created on: 24/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef ISAMSLAM_H_
#define ISAMSLAM_H_

#include <QList>
#include <isam/isam.h>
#include "isamlandmark.h"
#include "slamengine.h"
#include "scanmatcherselection.h"
#include "slam/scanmatching/scanmatcher.h"

namespace SLAM {
namespace Engine {

class ISAMRetriever;

class ISAMSLAM: public SLAMEngine
{
public:
    ISAMSLAM(uint robotId, const Data::Pose &initialPose,
            ScanMatcherSelection matcher = RANSACMatcherSelection);
    virtual ~ISAMSLAM();
    Map getMap() const;

public slots:
    void handleScan(double timestamp, const Geometry::PointScan &scan);
    void handleOdometry(double timestamp, const Data::Pose &pose);
    void handleINS(const Data::INSData &ins);

signals:
    void newRobotPose(TimedPose pose);

private:
    QList<int> lookupCovariancesAndLandmarks(
            const Geometry::SegmentScan &query, ISAMLandmark *robot);

    bool sufficientlyFar() const;
    bool sufficientlyFarFromStructural() const;
    Eigen::Matrix3d compactMeasureInformation(
            const ISAMLandmark *from, const Geometry::Rototranslation &to,
            const Eigen::MatrixXd &jointCov) const;
    void addLandmark(double timestamp, const Geometry::SegmentScan *scan = NULL);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    uint robotId;
    bool firstOdometry;
    Data::INSData ins;
    Geometry::Rototranslation initialPose, odometry, deltaOdometry;
    ScanMatching::ScanMatcher<ISAMRetriever> *matcher;
    QList<ISAMLandmark *> landmarks;
    QList<ISAMLandmark *> lastPoses;
    isam::Slam slam;
    AlignedVector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d> > covariances;
    Eigen::Matrix3d robotCovariance;
};

} /* namespace Engine */
} /* namespace SLAM */
#endif /* ISAMSLAM_H_ */
