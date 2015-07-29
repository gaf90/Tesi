/*
 * truthmap.h
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef TRUTHMAP_H_
#define TRUTHMAP_H_

#include "markedlinesegment.h"
#include "slam/geometry/segmentscan.h"
#include <QList>
#include <QString>

namespace SLAM {
namespace Evaluation {

class TruthMap
{
public:
    TruthMap();
    TruthMap(const QString &fname);
    ~TruthMap();

    void load(const QString &fname);
    void mapThinning();

    int nodeCount() const;
    const Data::Pose &pose(int i) const;
    const Geometry::SegmentScan *scan(int i) const;
    QList<Geometry::LineSegment *> map() const;

    QList<Geometry::LineSegment *> portion(const Geometry::Point &center, double radius) const;


private:
    QList<Geometry::SegmentScan *> scans;
    QList<Data::Pose> poses;
    QList<MarkedLineSegment *> segments;
};

} /* namespace Evaluation */
} /* namespace SLAM */
#endif /* TRUTHMAP_H_ */
