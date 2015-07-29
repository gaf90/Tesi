/*
 * truthmap.cpp
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#include "truthmap.h"
#include <QFile>
#include "data/laserdata.h"
#include "data/pose.h"
#include "slam/scanmatching/associationamigoni.h"

namespace SLAM {
namespace Evaluation {

using namespace ScanMatching;
using namespace Geometry;
using namespace Data;

TruthMap::TruthMap()
{
}

TruthMap::TruthMap(const QString &fname)
{
    load(fname);
}

TruthMap::~TruthMap()
{
    qDeleteAll(scans);
    qDeleteAll(segments);
}

void TruthMap::load(const QString &fname)
{
    qDeleteAll(scans);
    qDeleteAll(segments);
    scans.clear();
    poses.clear();
    segments.clear();

    QFile f(fname);
    f.open(QIODevice::ReadOnly);
    while(!f.atEnd()) {
        QString line = f.readLine();
        if(line.startsWith("FLASER")) {
            QList<double> readings;
            QStringList list = line.split(' ');
            int nreadings = list[1].toInt();
            for(int i = 2; i < nreadings + 2; i++) {
                readings.append(list[i].toDouble());
            }
            double timestamp = list[nreadings + 8].toDouble();
            double fov = M_PI;
            double resolution = fov / (nreadings - 1);
            double x = list[nreadings + 2].toDouble();
            double y = list[nreadings + 3].toDouble();
            double t = list[nreadings + 4].toDouble();
            PointScan scan(LaserData(timestamp, fov, resolution, readings));
            SegmentScan *ss = new SegmentScan(scan);
            Pose p(x, y, t - M_PI_2);
            Rototranslation rt(p);

            scans.append(ss);
            poses.append(p);
            int i = 0;
            fforeach(const UncertainLineSegment &s, ss->getSegments()) {
                MarkedLineSegment *mls = new MarkedLineSegment(ss, i++);
                static_cast<LineSegment &>(*mls) = rt * static_cast<const LineSegment &>(s);
                segments.append(mls);
            }
        }
    }
    f.close();

    mapThinning();
}

void TruthMap::mapThinning()
{
    static double angleThreshold = 5 * M_PI / 180;
    static double thinningThreshold = 0.15;
    static double overlapThreshold = 0.05;

    bool merged = true;
    while(merged) {
        merged = false;
        for(int i = 0; i < segments.size() - 1; i++) {
            for(int j = i + 1; j < segments.size(); j++) {
                MarkedLineSegment &lm = *(segments[i]), &ls = *(segments[j]);

                if(AssociationAmigoni::distance(lm, ls) < thinningThreshold &&
                        std::abs(linewrap(lm.angle() - ls.angle())) < angleThreshold &&
                        nonOverlapDistance(lm, ls) < overlapThreshold) {
                    /* Merge the line segments */
                    lm = lm.merge(ls);
                    delete &ls;
                    segments.removeAt(j);
                    j--;
                    merged = true;
                }
            }
        }
    }
}


int TruthMap::nodeCount() const
{
    return scans.size();
}

const Data::Pose &TruthMap::pose(int i) const
{
    return poses[i];
}

const Geometry::SegmentScan *TruthMap::scan(int i) const
{
    return scans[i];
}

QList<LineSegment *> TruthMap::map() const
{
    QList<LineSegment *> segs;
    fforeach(MarkedLineSegment *s, segments) {
        segs.append(s);
    }
    return segs;
}

QList<Geometry::LineSegment *> TruthMap::portion(
        const Geometry::Point &center, double radius) const
{
    QList<LineSegment *> segs;
    fforeach(MarkedLineSegment *s, segments) {
        if(s->intersectsCircle(center, radius)) {
            segs.append(s);
        }
    }
    return segs;
}

} /* namespace Evaluation */
} /* namespace SLAM */
