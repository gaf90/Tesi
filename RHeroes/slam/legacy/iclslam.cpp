/*
 * iclslam.cpp
 *
 *  Created on: 02/giu/2012
 *      Author: Mladen Mazuran
 */

#include "iclslam.h"
#include "slam/geometry/segmentscan.h"
#include "slam/legacy/associationelseberg.h"
#include "slam/legacy/associationamigoni.h"
#include "slam/legacy/minimizerdecoupled.h"
#include "slam/legacy/icl.h"
#include "shared/config.h"
#include "shared/random.h"
#include <QMultiMap>
#include <QThread>

#define MAX_SKIP_COUNT      10
#define BACKUP_TIME         30.

namespace SLAM {
namespace Legacy {

using namespace Data;
using namespace Support;
using namespace Geometry;
using namespace ScanMatching;
using namespace Shared;

#ifndef SLAM_SKIP_DEBUG
static int counter = 0;
#endif

typedef ICL<AssociationElseberg, MinimizerDecoupled, LineSegment> ICLImpl;

ICLSLAM::ICLSLAM(uint robotId, const Pose &initialPose) :
        robotId(robotId), firstScan(true), initialPose(initialPose), currentPose(),
    lastMeasurePose(), lastPose(), skipCount(0), primaryIdx(0), secondaryIdx(1), mapLock(false),
    outdoor(false)
{
}

ICLSLAM::~ICLSLAM()
{
    freeContent();
}

void ICLSLAM::freeContent()
{
    fforeach(LineSegment *s, segments) {
        delete s;
    }
    fforeach(LineSegment *s, obstacles) {
        delete s;
    }
    fforeach(Frontier *f, frontiers) {
        delete f;
    }
    segments.clear();
    obstacles.clear();
    frontiers.clear();
}


bool ICLSLAM::takeAMeasure(double timestamp)
{
    if(firstScan) {
        firstScan = false;
        return true;
    }

    bool timeCheck    = timestamp - lastMeasureTimestamp >=
                        SLAM_MIN_TEMPORAL_DISPLACEMENT;
    bool spatialCheck = lastPose.getDistance(lastMeasurePose) >=
                        SLAM_MIN_SPATIAL_DISPLACEMENT;
    bool angularCheck = std::fabs(lastMeasurePose.theta() - lastPose.theta()) >=
                        SLAM_MIN_ANGULAR_DISPLACEMENT;
#ifndef SLAM_SKIP_DEBUG
    ldbg << lastMeasurePose << "->" << lastPose << ": " << timeCheck << "," << spatialCheck << "," << angularCheck << endl;
#endif
    return timeCheck || spatialCheck || angularCheck;
}

bool ICLSLAM::doICL(double timestamp)
{
    if(firstScan) {
        /* Fake timestamp that forces scan integration after SLAM_START_TIME_SKIP seconds  */
        lastMeasureTimestamp = timestamp - (SLAM_MIN_TEMPORAL_DISPLACEMENT - SLAM_START_TIME_SKIP);
        lastThinningTimestamp = lastMeasureTimestamp;
        //firstScan = false;
        //currentPose = lastPose;

        return true;
    } else {
        return !mapLock; /* TODO: meno spesso */
    }
}


void ICLSLAM::mapThinning()
{
    for(int i = 0; i < segments.size() - 1; i++) {
        for(int j = i + 1; j < segments.size(); j++) {
            LineSegment &lm = *(segments[i]), &ls = *(segments[j]);
            double angle = linewrap(lm.angle() - ls.angle());

            /* Check if the segments are almost collinear */
            if(std::fabs(angle) < /*SM_COLLINEARITY_ANGLE_THRESHOLD / 2*/
                    Config::SLAM::collinearityThreshold) {
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

                        //ldbg << lm.length() << " " << ls.length() << endl;
                        //ldbg << lm.angle() << " " << ls.angle() << " ";
                        //ldbg << lm << " " << ls << endl;
                        /* Merge the line segments */
                        lm.mergeInPlace(ls);
                        //ldbg << lm.angle() << endl;
                        //ldbg << lm << endl;
                        delete &ls;
                        segments.removeAt(j);
                        j--;
                    }
                }
            }
        }
    }
}

void ICLSLAM::mergeSegments(const SegmentScan &s)
{
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
                        //lm.mergeInPlace(ls);
                        lm.mergeInPlaceProjection(ls);
                        break;
                    }
                }
            }
        }

        /* No segments were merged, add the last one to the map */
        if(j == segments.size() && scan[i].length() > SM_MINIMUM_SEGMENT_LENGTH) {
            LineSegment *s = new LineSegment(scan[i]);
            segments.append(s);
        }
    }
}


void ICLSLAM::mergeFrontiers(const SegmentScan &s)
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
        fforeach(const VisibilityPolygon *vp, visibilities) {
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

    visibilities.append(new VisibilityPolygon(scanPoly));
}

template <typename T>
static const LineSegment &getSegment(const QList<T> &l, int i)
{
    return l[i];
}

template <typename T>
static const LineSegment &getSegment(const QList<T *> &l, int i)
{
    return *l[i];
}

template <typename T>
static QString segmentPrefix(const QList<T> &l)
{
    return "";
}

template <typename T>
static QString segmentPrefix(const QList<T *> &l)
{
    return "new ";
}

template <typename T>
static void printMap(const QList<T> &l, const QString &name)
{
    QString xIni(""), xFin(""), yIni(""), yFin("");
    for(int i = 0; i < l.size(); i++) {
        const LineSegment &s = getSegment(l, i);
        xIni.append(" ").append(QString::number(s.x1(), 'f', 4));
        yIni.append(" ").append(QString::number(s.y1(), 'f', 4));
        xFin.append(" ").append(QString::number(s.x2(), 'f', 4));
        yFin.append(" ").append(QString::number(s.y2(), 'f', 4));
    }
    ldbg << "x" << name << "=[" << xIni << ";" << xFin << "];" << endl;
    ldbg << "y" << name << "=[" << yIni << ";" << yFin << "];" << endl;
}

template <typename T>
static void printMapCPP(const QList<T> &l, const QString &name)
{
    ldbg << name;
    for(int i = 0; i < l.size(); i++) {
        const LineSegment &s = getSegment(l, i);
        ldbg << endl << "<< " << segmentPrefix(l) << "LineSegment(" << s.x1() << "," << s.y1() << "," << s.x2() << "," << s.y2() << ")";
    }
    ldbg << ";" << endl;
}

static double poseDistance(const Eigen::Vector3d &z, const Data::Pose &odoGuess)
{
    const Eigen::Vector3d diff = odoGuess.vectorForm() - z;
    return SQUARE(diff[0]) + SQUARE(diff[1]) + std::abs(wrap(diff[2])) / (M_PI / 6);
}

void ICLSLAM::handleScan(double timestamp, const PointScan &pscan)
{
    doBackup(timestamp);

    if(doICL(timestamp)) {
        SegmentScan sm(pscan);
        bool skip = false;

        if(outdoor) {
            Rototranslation rt(currentPose);
            freeContent();
            fforeach(const LineSegment &s, sm.getSegments()) {
                segments.append(new LineSegment(rt * s));
            }
            fforeach(const Frontier &f, sm.getFrontiers()) {
                if(f.length() > 0.2) frontiers.append(new Frontier(rt * f));
            }
            if(takeAMeasure(timestamp)) {
                poses.append(TimedPose(timestamp, currentPose));
                lastMeasurePose = lastPose;
                lastMeasureTimestamp = timestamp;
            }
            return;
        }

        ICLImpl icl(sm, segments, currentPose);
        icl.run();
        Eigen::Vector3d z = icl.measure();

#ifndef SLAM_SKIP_DEBUG
        ldbg << "ins: " << ins.getPose() << endl;
        ldbg << "currentPose: " << currentPose << endl;
        ldbg << "z: " << z << endl;
#endif

        Eigen::Vector3d diff = currentPose.vectorForm() - z;
        if(std::abs(wrap(diff[2])) >= M_PI / 12 || SQUARE(diff[0]) + SQUARE(diff[1]) >= 3) {
            for(int i = 0; i < 10; i++) {
                Rototranslation perturbation(
                            Random::normal(0, 1 / 3.),
                            Random::normal(0, 1 / 3.),
                            Random::normal(0, SQUARE(M_PI / 6) / 3.));
                ICLImpl icl(sm, segments, perturbation * currentPose);
                icl.run();
                const Eigen::Vector3d &z1 = icl.measure();
                if(poseDistance(z1, currentPose) < poseDistance(z, currentPose)) {
#ifndef SLAM_SKIP_DEBUG
                    ldbg << "Cambiato" << endl;
#endif
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

#ifndef SLAM_SKIP_DEBUG
        ldbg << "ins: " << ins.getPose() << endl;
        ldbg << "currentPose: " << currentPose << endl;
        ldbg << "z: " << z << endl;
#endif

        Pose guess = currentPose;
        currentPose = z;

        if(!almostEqual(ins.getPose().theta(), 0, 0.02) ||
                 !almostEqual(ins.getPose().phi(), 0, 0.02)) {
            return;
        }

        SegmentScan rotoscan = Rototranslation(z) * sm;

#ifndef SLAM_SKIP_DEBUG
        ldbg << endl << "counter=" << counter++;
        ldbg << endl << "guess=" << guess << endl;
#endif
        //printMap(rotoscan.getSegments(), "scan");
        //printMap(segments, "walls");
        //printMap(frontiers, "frontiers");
        //ldbg << "plot(xwalls, ywalls, 'b', xfrontiers, yfrontiers, 'g', xscan, yscan, 'r');" << endl;
        //printMapCPP(sm.getSegments(), "scan");
        //printMapCPP(segments, "segments");

        //ldbg << "Graphics[" << rotoscan.getSegments() << ",Dashed," << rotoscan.getFrontiers() << "]" << endl;
        //ldbg << "Graphics[{" << segments << ",Dashed," << frontiers << ",Opacity[0.1],"
        //     << rotoscan.toPolygon() << "}]" << endl;


        mergeSegments(rotoscan);
        if(takeAMeasure(timestamp)) {
            mergeFrontiers(rotoscan);
            poses.append(TimedPose(timestamp, currentPose));
            lastMeasurePose = lastPose;
            lastMeasureTimestamp = timestamp;
        }
        if(timestamp - lastThinningTimestamp >= SLAM_MAP_THINNING_INTERVAL) {
#ifndef SLAM_SKIP_DEBUG
            ldbg << "Map Thinning" << endl;
#endif
            mapThinning();
            lastThinningTimestamp = timestamp;
        }

#ifndef SLAM_SKIP_DEBUG
        //ldbg << endl << "ListPlot[" << pscan << "]" << endl;
        ldbg << endl << "Graphics[" << sm.getSegments() << "]";
        ldbg << endl << "Graphics[{" << segments << ",Dashed," << frontiers << ",Opacity[0.1],"
             << rotoscan.toPolygon() << "}]" << endl;
#endif

        /*printMap(rotoscan.getSegments(), "scan");
        printMap(segments, "walls");
        printMap(frontiers, "frontiers");
        ldbg << "plot(xwalls, ywalls, 'b', xfrontiers, yfrontiers, 'g', xscan, yscan, 'r');" << endl;
        */

        //if(counter == 1508)
        //    exit(0);

        Pose p = Rototranslation(initialPose) * currentPose;
        emit newRobotPose(TimedPose(lastPose.timestamp(), p));
    }
}

void ICLSLAM::handleOdometry(double timestamp, const Data::Pose &pose)
{
    if(mapLock) return;

    if(firstScan) lastPose = TimedPose(timestamp, pose);
#ifndef SLAM_SKIP_DEBUG
    ldbg << "Delta Pose:" << pose.vectorForm() - lastPose.vectorForm() << endl;
#endif
    //currentPose = currentPose.vectorForm() + pose.vectorForm() - lastPose.vectorForm();
    currentPose = (Rototranslation(currentPose) * Rototranslation(lastPose).inverse() * Rototranslation(pose)).vectorForm();
    lastPose = TimedPose(timestamp, pose);
    emit newRobotPose(TimedPose(lastPose.timestamp(), Rototranslation(initialPose) * currentPose));
}

void ICLSLAM::handleINS(const Data::INSData &ins)
{
    this->ins = ins;
}

void ICLSLAM::handleMapKO()
{
    mapLock = true;

    fforeach(LineSegment *s, segments) delete s;
    fforeach(LineSegment *s, obstacles) delete s;
    fforeach(Frontier *f, frontiers) delete f;

    segments.clear();
    obstacles.clear();
    frontiers.clear();
    poses.clear();

    fforeach(const LineSegment &s, backupSegments[primaryIdx]) {
        segments.append(new LineSegment(s));
    }

    fforeach(const LineSegment &s, backupObstacles[primaryIdx]) {
        obstacles.append(new LineSegment(s));
    }

    fforeach(const TimedPose &p, backupPoses[primaryIdx]) {
        poses.append(TimedPose(p));
    }

    if(poses.size() > 0) {
        currentPose = poses.last();
    } else {
        currentPose = initialPose;
    }
}

void ICLSLAM::handleOutdoor()
{
    outdoor = true;
}

void ICLSLAM::doBackup(double timestamp)
{
    if(mapLock || timestamp - lastBackupTimestamp < BACKUP_TIME)
        return;

    backupSegments[primaryIdx].clear();
    backupObstacles[primaryIdx].clear();
    backupPoses[primaryIdx].clear();

    exchange(primaryIdx, secondaryIdx);

    fforeach(const LineSegment *s, segments) {
        backupSegments[secondaryIdx].append(LineSegment(*s));
    }

    fforeach(const LineSegment *s, obstacles) {
        backupObstacles[secondaryIdx].append(LineSegment(*s));
    }

    fforeach(const TimedPose &p, poses) {
        backupPoses[secondaryIdx].append(TimedPose(p));
    }

    lastBackupTimestamp = timestamp;
}

void ICLSLAM::handleSonar(const Geometry::LineSegment &s)
{
    Q_UNUSED(s)
    /*
    bool found = false;

    fforeach(LineSegment *seg, segments) {
        if(AssociationAmigoni<1>::distance(s, *seg) < 0.2) {
            found = true;
            seg->mergeInPlaceProjection(s);
            return;
        }
    }
    fforeach(LineSegment *seg, obstacles) {
        if(AssociationAmigoni::distance(s, *seg) < 0.2) {
            found = true;
            seg->mergeInPlace(s);
            return;
        }
    }
    obstacles.append(new LineSegment(s));
    */
}


Map ICLSLAM::getMap() const
{
    Rototranslation rt(initialPose);

    Map m;
    fforeach(const LineSegment *s, segments) {
        m.addWall(rt * (*s));
    }
    fforeach(const LineSegment *s, obstacles) {
        m.addObstacle(rt * (*s));
    }
    fforeach(const Frontier *f, frontiers) {
        m.addFrontier(rt * (*f));
    }
    fforeach(const TimedPose &p, poses) {
        Pose p1 = rt * p;
        m.addPose(robotId, TimedPose(p.timestamp(), p1));
    }
    Pose p1 = rt * currentPose;
    m.addPose(robotId, TimedPose(lastPose.timestamp(), p1));
    m.addPose(BASE_STATION_ID, TimedPose(0, Config::baseStationPose));

    return m;
}

} /* namespace Legacy */
} /* namespace SLAM */
