/*
 * iclslam.h
 *
 *  Created on: 07/giu/2012
 *      Author: Mladen Mazuran
 */

#ifndef LEGACY_ICLSLAM_H_
#define LEGACY_ICLSLAM_H_

#include "slam/engine/slamengine.h"
#include "slam/geometry/segmentscan.h"

namespace SLAM {
namespace Legacy {

class ICLSLAM : public Engine::SLAMEngine
{
    Q_OBJECT

public:
    ICLSLAM(uint robotId, const Data::Pose &initialPose);
    virtual ~ICLSLAM();

    bool takeAMeasure(double timestamp);
    bool doICL(double timestamp);

    Map getMap() const;

public slots:
    void handleScan(double timestamp, const Geometry::PointScan &scan);
    void handleOdometry(double timestamp, const Data::Pose &pose);
    void handleINS(const Data::INSData &ins);
    void handleSonar(const Geometry::LineSegment &s);
    void handleMapKO();
    void handleOutdoor();

//signals:
//    void newRobotPose(TimedPose pose);

private:
    void mapThinning();
    void mergeSegments(const Geometry::SegmentScan &s);
    void mergeFrontiers(const Geometry::SegmentScan &s);
    void doBackup(double timestamp);
    void freeContent();

private:
    uint robotId;
    bool firstScan;
    double lastMeasureTimestamp;    /* Timestamp of last measure */
    double lastThinningTimestamp;   /* Timestamp of last map thinning */
    double lastBackupTimestamp;
    Data::Pose initialPose, currentPose;
    Data::INSData ins;
    TimedPose lastMeasurePose;      /* Odometry pose during last measure */
    TimedPose lastPose;             /* Last odometry pose received */

    QList<Geometry::LineSegment *> segments, obstacles;
    QList<Geometry::Frontier *> frontiers;
    QList<Geometry::VisibilityPolygon *> visibilities;
    QList<TimedPose> poses;
    QList<Geometry::LineSegment> backupSegments[2], backupObstacles[2];
    QList<TimedPose> backupPoses[2];
    int skipCount, primaryIdx, secondaryIdx;
    bool mapLock, outdoor;

};

} /* namespace Legacy */
} /* namespace SLAM */
#endif /* LEGACY_ICLSLAM_H_ */
