/*
 * deterministicslam.h
 *
 *  Created on: 03/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef DETERMINISTICSLAM_H_
#define DETERMINISTICSLAM_H_

#include "slam/engine/slamengine.h"
#include "slam/geometry/segmentscan.h"
#include "slam/geometry/visibilitypolygon.h"
#include "slam/geometry/uncertainrototranslation.h"
#include "slam/scanmatching/scanmatcher.h"
#include "semideterministicretriever.h"
#include "scanmatcherselection.h"

namespace SLAM {
namespace Engine {

class DeterministicSLAM : public SLAMEngine
{
    Q_OBJECT

public:
    DeterministicSLAM(
            uint robotId, const Data::Pose &initialPose,
            ScanMatcherSelection matcher = RANSACMatcherSelection);
    virtual ~DeterministicSLAM();

    bool doScanMatching(double timestamp);
    bool takeAMeasure(SemiDeterministicRetriever &retriever, const QList<int> *associations);

    Map getMap() const;
    Map getMap(bool fullMap) const;
    void handleMapKO();
    void handleOutdoor();
    void mergeMap(const SLAM::Map& otherMap,uint mapId);

public slots:
    void handleScan(double timestamp, const Geometry::PointScan &scan);
    void handleOdometry(double timestamp, const Data::Pose &pose);
    void handleINS(const Data::INSData &ins);


private:
    void mapThinning();
    void mergeSegments(const Geometry::SegmentScan &s);
    void mergeOtherSegments(const Geometry::SegmentScan &s);
    void mergeFrontiers(const Geometry::SegmentScan &s);
    void mergeFrontiers(const Geometry::SegmentScan &s,QList<Geometry::Frontier*>& frontiers);
    void freeContent();
    void doBackup(double timestamp);

private:
    uint robotId;
    bool firstScan;
    double lastMeasureTimestamp;    /* Timestamp of last measure */
    double lastThinningTimestamp;   /* Timestamp of last map thinning */
    Data::Pose initialPose, currentPose;
    Data::INSData ins;
    TimedPose lastMeasurePose;      /* Odometry pose during last measure */
    TimedPose lastPose;             /* Last odometry pose received */
    Geometry::Rototranslation deltaOdometry;

    QList<Geometry::LineSegment *> segments;
    QList<Geometry::LineSegment *> otherSegments;
    QList<TimedPose> poses;
    QList<Geometry::VisibilityPolygon *> poseVisibilities;
    QList<Geometry::Frontier *> frontiers;

    int skipCount;

    ScanMatching::ScanMatcher<SemiDeterministicRetriever> *matcher;

    bool outdoor, mapLock;
    int primaryIdx, secondaryIdx;
    double lastBackupTimestamp;
    QList<Geometry::LineSegment *> backupSegments[2];
    QList<Geometry::VisibilityPolygon *> backupPoseVisibilities[2];
    QList<Geometry::Frontier *> backupFrontiers[2];
    QList<TimedPose> backupPoses[2];

    QMap<Geometry::LineSegment,int> mergedSegments;
    QMap<Geometry::Frontier,int> mergedFrontiers;

    QMap<uint,QList<Geometry::Frontier *> > otherRobotFrontiers;

};

} /* namespace Engine */
} /* namespace SLAM */
#endif /* DETERMINISTICSLAM_H_ */
