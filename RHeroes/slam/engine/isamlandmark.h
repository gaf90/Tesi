/*
 * isamlandmark.h
 *
 *  Created on: 26/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef ISAMLANDMARK_H_
#define ISAMLANDMARK_H_

#include <isam/slam2d.h>
#include <QSet>
#include "data/pose.h"
#include "slam/geometry/segmentscan.h"
#include "slam/geometry/scanbox.h"
#include "slam/geometry/visibilitypolygon.h"

namespace SLAM {
namespace Engine {

class ISAMLandmark: public isam::Pose2d_Node
{
public:
    enum LandmarkType {
        Structural,
        Transitional,
        Temporary
    };

    ISAMLandmark(uint id, double ts, LandmarkType type = Temporary);
    ISAMLandmark(uint id, double ts, const Geometry::SegmentScan &scan);

    void addCorrelation(ISAMLandmark *landmark);
    void propagate();

    double timestamp() const;
    const Geometry::ScanBox &box() const;
    Geometry::ScanBox boxWorld() const;
    const Geometry::VisibilityPolygon &visibility() const;
    const QSet<ISAMLandmark *> &correlations() const;
    Data::Pose pose() const;
    LandmarkType type() const;
    bool structural() const;
    bool transitional() const;
    bool temporary() const;
    uint id() const { return uid; }

    inline const AlignedVector<Geometry::UncertainLineSegment> &segments() const {
        return usegments;
    }
    inline const AlignedVector<Geometry::UncertainLineSegment> &propagatedSegments() const {
        return propagated;
    }

private:
    uint uid;
    double ts;
    LandmarkType t;
    AlignedVector<Geometry::UncertainLineSegment> usegments, propagated;
    Geometry::ScanBox sbox;
    Geometry::VisibilityPolygon poly;
    QSet<ISAMLandmark *> correlated;
};

} /* namespace Engine */
} /* namespace SLAM */
#endif /* ISAMLANDMARK_H_ */
