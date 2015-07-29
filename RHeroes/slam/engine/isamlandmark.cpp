/*
 * isamlandmark.cpp
 *
 *  Created on: 26/gen/2013
 *      Author: Mladen Mazuran
 */

#include "isamlandmark.h"

namespace SLAM {
namespace Engine {

ISAMLandmark::ISAMLandmark(uint id, double ts, ISAMLandmark::LandmarkType t) :
        uid(id), ts(ts), t(t)
{
}

ISAMLandmark::ISAMLandmark(uint id, double ts, const Geometry::SegmentScan &scan) :
        uid(id), ts(ts), t(Structural), usegments(scan.getSegments()),
        propagated(scan.getSegments().size()), sbox(scan), poly(scan.toPolygon())
{
}

Data::Pose ISAMLandmark::pose() const
{
    isam::Pose2d p = value();
    return Data::Pose(p.x(), p.y(), p.t());
}

void ISAMLandmark::propagate()
{
    Geometry::Rototranslation rt(pose());
    for(int i = 0; i < usegments.size(); i++) {
        propagated[i] = rt * usegments[i];
    }
}

void ISAMLandmark::addCorrelation(ISAMLandmark *landmark)
{
    correlated.insert(landmark);
}

const Geometry::VisibilityPolygon &ISAMLandmark::visibility() const
{
    return poly;
}

double ISAMLandmark::timestamp() const
{
    return ts;
}

const QSet<ISAMLandmark *> &ISAMLandmark::correlations() const
{
    return correlated;
}

const Geometry::ScanBox &ISAMLandmark::box() const
{
    return sbox;
}

Geometry::ScanBox ISAMLandmark::boxWorld() const
{
    return Geometry::Rototranslation(value().vector()) * sbox;
}

ISAMLandmark::LandmarkType ISAMLandmark::type() const
{
    return t;
}

bool ISAMLandmark::structural() const
{
    return t == Structural;
}

bool ISAMLandmark::transitional() const
{
    return t == Transitional;
}

bool ISAMLandmark::temporary() const
{
    return t == Temporary;
}


} /* namespace Engine */
} /* namespace SLAM */
