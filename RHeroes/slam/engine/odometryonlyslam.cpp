/*
 * odometryonlyslam.cpp
 *
 *  Created on: 11/dic/2012
 *      Author: Mladen Mazuran
 */

#include "odometryonlyslam.h"
#include "slam/geometry/segmentscan.h"
#include "shared/config.h"
#include "shared/random.h"

namespace SLAM {
namespace Engine {

using namespace Data;
using namespace Support;
using namespace Geometry;
using namespace Shared;

OdometryOnlySLAM::OdometryOnlySLAM(uint robotId, const Pose &initialPose) :
        robotId(robotId), firstScan(true), initialPose(initialPose), currentPose(),
        lastMeasurePose(), lastPose()
{
}

OdometryOnlySLAM::~OdometryOnlySLAM()
{
    freeContent();
}

void OdometryOnlySLAM::freeContent()
{
    qDeleteAll(segments);
    qDeleteAll(poseVisibilities);
}

bool OdometryOnlySLAM::takeAMeasure()
{
    if(firstScan) {
        firstScan = false;
        return true;
    }

    bool spatialCheck = lastPose.getDistance(lastMeasurePose) >=
                        SLAM_MIN_SPATIAL_DISPLACEMENT;
    bool angularCheck = std::fabs(lastMeasurePose.theta() - lastPose.theta()) >=
                        SLAM_MIN_ANGULAR_DISPLACEMENT;

    return spatialCheck || angularCheck;
}

void OdometryOnlySLAM::mergeSegments(const SegmentScan &s)
{
    const AlignedVector<UncertainLineSegment> &scan = s.getSegments();
    for(int i = 0; i < scan.size(); i++) {
        LineSegment *s = new LineSegment(scan[i]);
        segments.append(s);
    }
}

void OdometryOnlySLAM::handleScan(double timestamp, const PointScan &pscan)
{
    if(!almostEqual(ins.getPose().theta(), 0, 0.02) ||
             !almostEqual(ins.getPose().phi(), 0, 0.02)) {
        return;
    }

    if(takeAMeasure()) {
        SegmentScan sm(pscan);
        SegmentScan rotoscan = Rototranslation(currentPose) * sm;
        mergeSegments(rotoscan);
        poses.append(TimedPose(timestamp, currentPose));
        lastMeasurePose = lastPose;
        lastMeasureTimestamp = timestamp;
        poseVisibilities.append(new VisibilityPolygon(rotoscan.toPolygon()));
    }

    Pose p = Rototranslation(initialPose) * currentPose;
    emit newRobotPose(TimedPose(lastPose.timestamp(), p));
}

void OdometryOnlySLAM::handleOdometry(double timestamp, const Data::Pose &pose)
{
    if(firstScan) lastPose = TimedPose(timestamp, pose);

    currentPose = pose.vectorForm(); //(Rototranslation(currentPose) * Rototranslation(lastPose).inverse() * Rototranslation(pose)).vectorForm();
    lastPose = TimedPose(timestamp, pose);
    emit newRobotPose(TimedPose(lastPose.timestamp(), Rototranslation(initialPose) * currentPose));
}

void OdometryOnlySLAM::handleINS(const Data::INSData &ins)
{
    this->ins = ins;
}

Map OdometryOnlySLAM::getMap() const
{
    int i = 0;
    Rototranslation rt(initialPose);

    Map m;
    fforeach(const LineSegment *s, segments) {
        m.addWall(rt * (*s));
    }
    fforeach(const TimedPose &p, poses) {
        Pose p1 = rt * p;
        m.addPose(robotId, TimedPose(p.timestamp(), p1));
        const_cast<PathNode *>(m.lastRobotPose(robotId))->setVisibility(poseVisibilities[i++]);
    }
    Pose p1 = rt * currentPose;
    m.addPose(robotId, TimedPose(lastPose.timestamp(), p1));
    m.addPose(BASE_STATION_ID, TimedPose(0, Config::baseStationPose));

    return m;
}

} /* namespace Engine */
} /* namespace SLAM */
