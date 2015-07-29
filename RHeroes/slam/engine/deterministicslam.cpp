/*
 * deterministicslam.cpp
 *
 *  Created on: 03/dic/2012
 *      Author: Mladen Mazuran
 */

#include "deterministicslam.h"
#include "odometrycovariancemodels.h"
#include "slam/geometry/segmentscan.h"
#include "slam/scanmatching/associationelseberg.h"
#include "slam/scanmatching/associationposecentric.h"
#include "slam/scanmatching/associationprobabilistic.h"
#include "slam/scanmatching/decoupledestimator.h"
#include "slam/scanmatching/filteredicl.h"
#include "slam/scanmatching/ransacmatcher.h"
#include "slam/scanmatching/ligriffithsicl.h"
#include "slam/scanmatching/classicicl.h"
#include "shared/config.h"
#include "shared/random.h"
#include <QThread>
#include <QTime>
#include <Eigen/Cholesky>

#define MAX_SKIP_COUNT      10
#define BACKUP_TIME         10.0f

namespace SLAM {
namespace Engine {

using namespace Data;
using namespace Support;
using namespace Geometry;
using namespace ScanMatching;
using namespace Shared;

typedef BasicOdometryCovarianceModel OdometryCovarianceModel;
typedef AssociationPoseCentric AssociationModel;

DeterministicSLAM::DeterministicSLAM(
        uint robotId, const Pose &initialPose, ScanMatcherSelection matcher) :
        robotId(robotId), firstScan(true), initialPose(initialPose), currentPose(),
        lastMeasurePose(), lastPose(), deltaOdometry(),
        skipCount(0),outdoor(false), mapLock(false),lastBackupTimestamp(0.0f),primaryIdx(0), secondaryIdx(1)
{
    switch(matcher) {
    case LiGriffithsICLSelection:
        this->matcher = new LiGriffithsICL<SemiDeterministicRetriever>;
        break;
    case ClassicICLSelection:
        this->matcher = new ClassicICL<SemiDeterministicRetriever, AssociationModel>;
        break;
    case FilteredICLSelection:
        this->matcher = new FilteredICL<SemiDeterministicRetriever, AssociationModel>;
        break;
    case RANSACMatcherSelection:
        this->matcher = new RANSACMatcher<SemiDeterministicRetriever, AssociationModel>;
        break;
    }
}

DeterministicSLAM::~DeterministicSLAM()
{
    freeContent();
}

void DeterministicSLAM::freeContent()
{
    qDeleteAll(segments);
    qDeleteAll(poseVisibilities);
    qDeleteAll(frontiers);
    delete matcher;
}

bool DeterministicSLAM::takeAMeasure(
        SemiDeterministicRetriever &retriever, const QList<int> *associations)
{
    if(firstScan) {
        firstScan = false;
        return true;
    }
#if OVERLAP_BASED_MEASURE
    double scanSegmentLength = 0;
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        scanSegmentLength += retriever.querySegment(i).length();
    }
    return ScanMatching::overlapAmount(
            retriever, currentPose, associations) < .3 * scanSegmentLength;
#else
    Q_UNUSED(retriever) Q_UNUSED(associations)
    bool spatialCheck = lastPose.getDistance(lastMeasurePose) >=
                        SLAM_MIN_SPATIAL_DISPLACEMENT;
    bool angularCheck = std::fabs(lastMeasurePose.theta() - lastPose.theta()) >=
                        SLAM_MIN_ANGULAR_DISPLACEMENT;

    return spatialCheck || angularCheck;
#endif
}

bool DeterministicSLAM::doScanMatching(double timestamp)
{
    if(firstScan) {
        /* Fake timestamp that forces scan integration after SLAM_START_TIME_SKIP seconds  */
        lastMeasureTimestamp = timestamp - (SLAM_MIN_TEMPORAL_DISPLACEMENT - SLAM_START_TIME_SKIP);
        lastThinningTimestamp = lastMeasureTimestamp;
        //firstScan = false;
        //currentPose = lastPose;

        return true;
    } else {
        return true;
    }
}


void DeterministicSLAM::mapThinning()
{
    for(int i = 0; i < segments.size() - 1; i++) {
        for(int j = i + 1; j < segments.size(); j++) {
            LineSegment &lm = *(segments[i]), &ls = *(segments[j]);
            double angle = linewrap(lm.angle() - ls.angle());

            /* Check if the segments are almost collinear */
            if(std::fabs(angle) < Config::SLAM::collinearityThreshold) {
                /* Get a new line segment by rotating around the centroid in order to align
                   the two segments */
                LineSegment rotated = Rototranslation(ls.centroid(), angle) * ls;
                const Point &rp1 = rotated.p1();
                const Point &rp2 = rotated.p2();

                /* Check if the point to line distance of one (either one is ok) of the
                   rotated segment is small enough */
                if(lm.distance2(rp1) < SQUARE(Config::SLAM::thinningThreshold)) {
                    /* Check that the projection of at least one the endpoints falls in an
                       acceptable range */
                    double proj1 = lm.project1D(rp1), proj2 = lm.project1D(rp2);
                    double accept_l = - Config::SLAM::mergeThreshold;
                    double accept_u = lm.length() + Config::SLAM::mergeThreshold;

                    /* TODO: Serve che il controllo sulla distanza abbia isolinee circolari,
                             non rettangolari come ora */
                    if((proj1 >= accept_l && proj1 <= accept_u) ||
                            (proj2 >= accept_l && proj2 <= accept_u)) {

                        /* Merge the line segments */
                        lm.mergeInPlace(ls);
                        delete &ls;
                        segments.removeAt(j);
                        j--;
                    }
                }
            }
        }
    }
}

void DeterministicSLAM::mergeSegments(const SegmentScan &s)
{
    QList<Geometry::LineSegment *> outdoorSegments;
    const AlignedVector<UncertainLineSegment> &scan = s.getSegments();
    int i, j;
    /* Slow O(n^2) search for line segments to merge */
    for(i = 0; i < scan.size(); i++) {
        for(j = 0; j < segments.size(); j++) {
            LineSegment &lm = *(segments[j]);
            const UncertainLineSegment &ls = scan[i];
            double angle = linewrap(lm.angle() - ls.angle());

            /* Check if the segments are almost collinear (and long enough) */
            if(lm.length() > 0.1 && ls.length() > 0.1 &&
                    std::fabs(angle) < Config::SLAM::collinearityThreshold) {
                /* Get a new line segment by rotating around the centroid in order to align
                   the two segments */
                LineSegment rotated = Rototranslation(ls.centroid(), angle) * ls;
                const Point &rp1 = rotated.p1();
                const Point &rp2 = rotated.p2();

                /* Check if the point to line distance of one (either one is ok) of the
                   rotated segment is small enough */
                if(lm.distance2(rp1) < SQUARE(Config::SLAM::mergeThreshold)) {
                    /* Check that the projection of at least one the endpoints falls in an
                       acceptable range */
                    double proj1 = lm.project1D(rp1), proj2 = lm.project1D(rp2);
                    double accept_l = - Config::SLAM::mergeThreshold;
                    double accept_u = lm.length() + Config::SLAM::mergeThreshold;

                    /* TODO: Serve che il controllo sulla distanza abbia isolinee circolari,
                             non rettangolari come ora */
                    if((proj1 >= accept_l && proj1 <= accept_u) ||
                            (proj2 >= accept_l && proj2 <= accept_u)) {

                        /* Merge the line segments */
                        lm.mergeInPlace(ls);
                        outdoorSegments.append(new LineSegment(lm));
                        //lm.mergeInPlaceProjection(ls);
                        break;
                    }
                }
            }
        }

        /* No segments were merged, add the last one to the map */
        if(j == segments.size() && scan[i].length() > SM_MINIMUM_SEGMENT_LENGTH) {
            LineSegment *s = new LineSegment(scan[i]);
            segments.append(s);
            outdoorSegments.append(new LineSegment(scan[i]));
        }
    }
    if(outdoor){
        segments=outdoorSegments;
    }
}

void DeterministicSLAM::mergeOtherSegments(const SegmentScan &s)
{
    //??QList<Geometry::LineSegment *> outdoorSegments;
    const AlignedVector<UncertainLineSegment> &scan = s.getSegments();
    int i, j;
    /* Slow O(n^2) search for line segments to merge */
    for(i = 0; i < scan.size(); i++) {
        for(j = 0; j < otherSegments.size(); j++) {
            LineSegment &lm = *(otherSegments[j]);
            const UncertainLineSegment &ls = scan[i];
            double angle = linewrap(lm.angle() - ls.angle());

            /* Check if the segments are almost collinear (and long enough) */
            if(lm.length() > 0.1 && ls.length() > 0.1 &&
                    std::fabs(angle) < Config::SLAM::collinearityThreshold) {
                /* Get a new line segment by rotating around the centroid in order to align
                   the two segments */
                LineSegment rotated = Rototranslation(ls.centroid(), angle) * ls;
                const Point &rp1 = rotated.p1();
                const Point &rp2 = rotated.p2();

                /* Check if the point to line distance of one (either one is ok) of the
                   rotated segment is small enough */
                if(lm.distance2(rp1) < SQUARE(Config::SLAM::mergeThreshold)) {
                    /* Check that the projection of at least one the endpoints falls in an
                       acceptable range */
                    double proj1 = lm.project1D(rp1), proj2 = lm.project1D(rp2);
                    double accept_l = - Config::SLAM::mergeThreshold;
                    double accept_u = lm.length() + Config::SLAM::mergeThreshold;

                    /* TODO: Serve che il controllo sulla distanza abbia isolinee circolari,
                             non rettangolari come ora */
                    if((proj1 >= accept_l && proj1 <= accept_u) ||
                            (proj2 >= accept_l && proj2 <= accept_u)) {

                        /* Merge the line segments */
                        lm.mergeInPlace(ls);
                        //???outdoorSegments.append(new LineSegment(lm));
                        //lm.mergeInPlaceProjection(ls);
                        break;
                    }
                }
            }
        }

        /* No segments were merged, add the last one to the map */
        if(j == otherSegments.size() ){//&& scan[i].length() > SM_MINIMUM_SEGMENT_LENGTH) {
            LineSegment *s = new LineSegment(scan[i]);
            otherSegments.append(s);
            //??outdoorSegments.append(new LineSegment(scan[i]));
        }
    }
    /*if(outdoor){
        segments=outdoorSegments;
    }*/
}

#ifdef PERTURBATION_RETRY
static double poseDistance(const Eigen::Vector3d &z, const Data::Pose &odoGuess)
{
    const Eigen::Vector3d diff = odoGuess.vectorForm() - z;
    return SQUARE(diff[0]) + SQUARE(diff[1]) + std::abs(wrap(diff[2])) / (M_PI / 6);
}
#endif

void DeterministicSLAM::handleScan(double timestamp, const PointScan &pscan)
{
    doBackup(timestamp);
    if(doScanMatching(timestamp)) {
        SegmentScan sm(pscan, SegmentScan::SplitAndMergeInterpolation);
        SemiDeterministicRetriever dr(segments, sm, currentPose *
                OdometryCovarianceModel::addCovariance(deltaOdometry));

#ifdef PERTURBATION_RETRY
        bool skip = false;
#endif

        matcher->setRetriever(dr);
        matcher->run();
        Eigen::Vector3d z = matcher->measure();
#ifdef PERTURBATION_RETRY
        Eigen::Vector3d diff = currentPose.vectorForm() - z;
        if(std::abs(wrap(diff[2])) >= M_PI / 12 || SQUARE(diff[0]) + SQUARE(diff[1]) >= 3) {
            for(int i = 0; i < 10; i++) {
                Rototranslation perturbation(
                            Random::normal(0, 1 / 3.),
                            Random::normal(0, 1 / 3.),
                            Random::normal(0, SQUARE(M_PI / 6) / 3.));
                dr = DeterministicRetriever(segments, sm, perturbation * currentPose);
                matcher->setRetriever(dr);
                matcher->run();
                const Eigen::Vector3d &z1 = matcher->measure();
                if(poseDistance(z1, currentPose) < poseDistance(z, currentPose)) {
                    z = z1;
                }
            }
        }

        diff = currentPose.vectorForm() - z;
        if(std::abs(wrap(diff[2])) >= M_PI / 12 || SQUARE(diff[0]) + SQUARE(diff[1]) >= 3) {
            z = currentPose;
            skipCount++;
            skip = true;
        }

        if(skip && skipCount <= MAX_SKIP_COUNT) {
            return;
        } else {
            skipCount = 0;
        }
#endif
        Pose guess = currentPose;
        currentPose = z;
        deltaOdometry = Rototranslation();

        if(!almostEqual(ins.getPose().theta(), 0, 0.02) ||
                 !almostEqual(ins.getPose().phi(), 0, 0.02)) {
            return;
        }

        if(takeAMeasure(dr, matcher->associations())) {
            SegmentScan rotoscan = Rototranslation(z) * sm;
            mergeSegments(rotoscan);
            mergeOtherSegments(rotoscan);
            if(outdoor){
                qDebug("SONO IN OUTDOOR!!!");
                frontiers.clear();
            }
            else{
                mergeFrontiers(rotoscan);
            }
            poses.append(TimedPose(timestamp, currentPose));
            lastMeasurePose = lastPose;
            lastMeasureTimestamp = timestamp;
            if(outdoor){
                poseVisibilities.clear();
            }
            while(poseVisibilities.size()<poses.size()){
                poseVisibilities.append(new VisibilityPolygon(rotoscan.toPolygon()));
            }
        } else {
            /* ? */
        }

        if(timestamp - lastThinningTimestamp >= SLAM_MAP_THINNING_INTERVAL) {
            mapThinning();
            lastThinningTimestamp = timestamp;
        }


        Pose p = Rototranslation(initialPose) * currentPose;
        emit newRobotPose(TimedPose(lastPose.timestamp(), p));
    }
}

void DeterministicSLAM::handleOdometry(double timestamp, const Data::Pose &pose)
{
    if(firstScan) lastPose = TimedPose(timestamp, pose);
    deltaOdometry = deltaOdometry * Rototranslation(lastPose).inverse() * Rototranslation(pose);
    //currentPose = (Rototranslation(currentPose) * Rototranslation(lastPose).inverse() * Rototranslation(pose)).vectorForm();
    lastPose = TimedPose(timestamp, pose);
    emit newRobotPose(TimedPose(lastPose.timestamp(), Rototranslation(initialPose) * currentPose));
}

void DeterministicSLAM::handleINS(const Data::INSData &ins)
{
    this->ins = ins;
}
Map DeterministicSLAM::getMap() const{
    return getMap(false);
}

Map DeterministicSLAM::getMap(bool fullMap) const
{
    int i = 0;
    Rototranslation rt(initialPose);

    Map m;
    QList <LineSegment* > mapSegments;
    if(fullMap){
        mapSegments=otherSegments;
    }
    else{
        mapSegments=segments;
    }
    fforeach(const LineSegment *s, mapSegments) {
       m.addWall(rt * (*s));
    }    
    fforeach(const Frontier *f, frontiers) {
        m.addFrontier(rt * (*f));
    }
    if(fullMap){
        foreach(QList< Frontier* > listFrontiers,otherRobotFrontiers){
            foreach(const Frontier *f, listFrontiers){
                 m.addFrontier(rt * (*f));
            }
        }
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

void DeterministicSLAM::mergeMap(const SLAM::Map &otherMap,uint mapId){

    QTime t;
    t.start();

    qDebug("Merge start!");
    const QList<LineSegment*> scan=otherMap.walls();
    Rototranslation rt(initialPose);
    AlignedVector<UncertainLineSegment> newSegments;
    int count=0;
    int total=0;
    foreach(LineSegment* segment,scan){
        if(!mergedSegments.contains(*segment)){
            mergedSegments.insert(*segment,42);
            newSegments.append(UncertainLineSegment(rt.inverse()* *segment,Eigen::Matrix4d()));
            count++;
        }
        total++;
    }
    QList<Frontier> newFrontiers;

    foreach(Frontier f,otherMap.getFrontiers()){
        //if(!mergedFrontiers.contains(f)){
            //mergedFrontiers.insert(f,42);
            newFrontiers.append(rt.inverse() * f);
            count++;
        //}
        total++;
    }
    qDebug("%i out of %i",count,total);
    qDebug("Partial time elapsed: %d ms", t.elapsed());
    SegmentScan s=SegmentScan(newSegments,newFrontiers);
    mergeOtherSegments(s);
    qDebug("Time elapsed0: %d ms", t.elapsed());
    otherRobotFrontiers[mapId].clear();
    mergeFrontiers(s,otherRobotFrontiers[mapId]);
    qDebug("Frontiers size: %i",otherRobotFrontiers[mapId].size());
    qDebug("Time elapsed: %d ms", t.elapsed());
}


void DeterministicSLAM::mergeFrontiers(const SegmentScan &s,QList<Frontier*>& otherFrontiers)
{
    const VisibilityPolygon scanPoly = s.toPolygon();

    /* Add the new frontiers, taking into account the new visibility polygon */
    fforeach(const Frontier &f, s.getFrontiers()) {
        LineSegment diff = f;
        fforeach(const VisibilityPolygon *vp, poseVisibilities) {
            diff = vp->lineDifference(diff);
            if(diff.length() < SM_MINIMUM_FRONTIER_LENGTH)
                break;
        }
        if(diff.length() >= SM_MINIMUM_FRONTIER_LENGTH) {
            Point p(currentPose.x(), currentPose.y());
            otherFrontiers.append(new Frontier(diff));
        }
    }



    /* Remove frontiers "behind" walls */
    for(int i = 0; i < otherFrontiers.size(); i++) {
        Frontier *f = otherFrontiers[i];
        fforeach(const LineSegment *s, segments) {
            if(AssociationAmigoni::distance(*s, *f) < 0.5) {
                const double l = f->length();
                double p1 = f->project1D(s->p1()), p2 = f->project1D(s->p2());
                if(p2 < p1) exchange(p1, p2);
                if(p1 > l || p2 < 0) {
                    continue;
                } else {
                    if(p1 > 0 && p2 < l) {
                        otherFrontiers.append(new Frontier(f->portion(p2, l)));
                        *f = f->portion(0, p1);
                    } else if(p1 > 0) {
                        *f = f->portion(0, p1);
                    } else if(p2 < l) {
                        *f = f->portion(p2, l);
                    } else {
                        otherFrontiers.removeAt(i);
                        i--;
                        delete f;
                        break;
                    }
                }
            }
        }
    }
}


void DeterministicSLAM::mergeFrontiers(const SegmentScan &s)
{
    const VisibilityPolygon scanPoly = s.toPolygon();

    /* Remove portions of the frontiers in the map taking into account the new visibility polygon */
    for(int i = 0; i < frontiers.size(); i++) {
        Frontier *f = frontiers[i];
        LineSegment diff = scanPoly.lineDifference(*f);
        if(diff.length2() < f->length2() - 1e-6) {
            delete f;
            if(diff.length() >= SM_MINIMUM_FRONTIER_LENGTH) {
                /* Replace */
                frontiers[i] = new Frontier(diff);
            } else {
                /* Delete */
                frontiers.removeAt(i);
                i--;
            }
        }
    }

    /* Add the new frontiers, taking into account the new visibility polygon */
    fforeach(const Frontier &f, s.getFrontiers()) {
        LineSegment diff = f;
        fforeach(const VisibilityPolygon *vp, poseVisibilities) {
            diff = vp->lineDifference(diff);
            if(diff.length() < SM_MINIMUM_FRONTIER_LENGTH)
                break;
        }
        if(diff.length() >= SM_MINIMUM_FRONTIER_LENGTH) {
            Point p(currentPose.x(), currentPose.y());
            frontiers.append(new Frontier(diff));
        }
    }

    /* Merge almost collinear frontiers */
    for(int i = 0; i < frontiers.size() - 1; i++) {
        Frontier *fi = frontiers[i];
        for(int j = i + 1; j < frontiers.size(); j++) {
            Frontier *fj = frontiers[j];

            if(std::fabs(wrap(fi->angle() - fj->angle())) < Config::SLAM::collinearityThreshold &&
               (fi->p2().distance2(fj->p1()) < 1e-8 ||
               fi->p2().distance2(fj->p2()) < 1e-8 ||
               fi->p1().distance2(fj->p1()) < 1e-8 ||
               fi->p1().distance2(fj->p2()) < 1e-8)) {
                const LineSegment merge = fi->merge(*fj);
                const Point points[] = { fi->p1(), fi->p2(), fj->p1(), fj->p2() };
                TopValues<1> tvmin, tvmax;

                for(int k = 0; k < 4; k++) {
                    const double proj = merge.project1D(points[k]);
                    tvmin.add(proj, k); tvmax.add(-proj, k);
                }

                const Point centroid = .5 * (points[tvmin.value()] + points[tvmax.value()]);
                if(fi->atRight(centroid) && fj->atRight(centroid)) {
                    *fi = LineSegment(points[tvmin.value()], points[tvmax.value()]);

                    delete fj;
                    frontiers.removeAt(j);
                    j--;
                }
            }
        }
    }

    /* Remove frontiers "behind" walls */
    for(int i = 0; i < frontiers.size(); i++) {
        Frontier *f = frontiers[i];
        fforeach(const LineSegment *s, segments) {
            if(AssociationAmigoni::distance(*s, *f) < 0.5) {
                const double l = f->length();
                double p1 = f->project1D(s->p1()), p2 = f->project1D(s->p2());
                if(p2 < p1) exchange(p1, p2);
                if(p1 > l || p2 < 0) {
                    continue;
                } else {
                    if(p1 > 0 && p2 < l) {
                        frontiers.append(new Frontier(f->portion(p2, l)));
                        *f = f->portion(0, p1);
                    } else if(p1 > 0) {
                        *f = f->portion(0, p1);
                    } else if(p2 < l) {
                        *f = f->portion(p2, l);
                    } else {
                        frontiers.removeAt(i);
                        i--;
                        delete f;
                        break;
                    }
                }
            }
        }
    }
}

void DeterministicSLAM::handleMapKO(){
    //??? Check
    mapLock = true;

    //Should be useless
    /*fforeach(LineSegment *s, segments) delete s;
    fforeach(Frontier *f, frontiers) delete f;*/

    //currentPose = poses.last();

    /*segments = backupSegments[primaryIdx];
    frontiers = backupFrontiers[primaryIdx];
    poses = backupPoses[primaryIdx];
    poseVisibilities = backupPoseVisibilities[primaryIdx];*/


    segments.clear();
    frontiers.clear();
    poses.clear();
    poseVisibilities.clear();
    //poses.append(currentPose);



    /*if(poses.size() > 0) {
        currentPose = poses.last();
    } else {
        currentPose = initialPose;
    }*/

}

void DeterministicSLAM::handleOutdoor(){
    outdoor=true;
}

void DeterministicSLAM::doBackup(double timestamp){
    if(mapLock || timestamp - lastBackupTimestamp < BACKUP_TIME){
        return;
    }

    backupPoses[primaryIdx]=QList<TimedPose>(poses);
    backupFrontiers[primaryIdx]=QList<Geometry::Frontier *>(frontiers);
    backupPoseVisibilities[primaryIdx]=QList<Geometry::VisibilityPolygon *>(poseVisibilities);
    backupSegments[primaryIdx]=QList<Geometry::LineSegment *>(segments);

    exchange(primaryIdx,secondaryIdx);

    lastBackupTimestamp = timestamp;
}

} /* namespace Engine */
} /* namespace SLAM */
