/*
 * linesegment.cpp
 *
 *  Created on: 08/mar/2012
 *      Author: Mladen Mazuran
 */

#include "linesegment.h"
#include "slam/constants.h"

namespace SLAM {
namespace Geometry {

void LineSegment::serializeTo(QDataStream &stream) const
{
    stream << p1v << p2v;
}

void LineSegment::deserializeFrom(QDataStream &stream)
{
    stream >> p1v >> p2v;
}

LoggerStream &operator<<(LoggerStream &stream, const LineSegment &line)
{
#ifdef FMT_MATHEMATICA
    return stream << "Line[{" << line.p1() << "," << line.p2() << "}]";
#else
    return stream << "Line(" << line.p1() << "->" << line.p2() << ")";
#endif
}

LoggerStream &operator<<(LoggerStream &stream, const LineSegment *line)
{
    return stream << *line;
}


} /* namespace Geometry */
} /* namespace SLAM */
