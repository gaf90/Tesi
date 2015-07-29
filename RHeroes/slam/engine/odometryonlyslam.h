/*
 * odometryonlyslam.h
 *
 *  Created on: 03/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef ODOMETRYONLYSLAM_H_
#define ODOMETRYONLYSLAM_H_

#include "slam/engine/slamengine.h"
#include "slam/geometry/segmentscan.h"
#include "slam/geometry/visibilitypolygon.h"

namespace SLAM {
namespace Engine {

class OdometryOnlySLAM : public SLAMEngine
{
    Q_OBJECT

public:
    OdometryOnlySLAM(uint robotId, const Data::Pose &initialPose);
    virtual ~OdometryOnlySLAM();

    bool takeAMeasure();
    Map getMap() const;

public slots:
    void handleScan(double timestamp, const Geometry::PointScan &scan);
    void handleOdometry(double timestamp, const Data::Pose &pose);
    void handleINS(const Data::INSData &ins);

signals:
    void newRobotPose(TimedPose pose);

private:
    void mergeSegments(const Geometry::SegmentScan &s);
    void freeContent();

private:
    uint robotId;
    bool firstScan;
    double lastMeasureTimestamp;    /* Timestamp of last measure */
    Data::Pose initialPose, currentPose;
    Data::INSData ins;
    TimedPose lastMeasurePose;      /* Odometry pose during last measure */
    TimedPose lastPose;             /* Last odometry pose received */

    QList<Geometry::LineSegment *> segments;
    QList<TimedPose> poses;
    QList<Geometry::VisibilityPolygon *> poseVisibilities;
};

} /* namespace Engine */
} /* namespace SLAM */
#endif /* ODOMETRYONLYSLAM_H_ */
