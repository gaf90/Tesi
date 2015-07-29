/*
 * scanbox.h
 *
 *  Created on: 11/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef SCANBOX_H_
#define SCANBOX_H_

#include "point.h"
#include "quadrilateral.h"
#include "segmentscan.h"
#include "rototranslation.h"
#include <Eigen/Core>
#include <deque>

namespace SLAM {
namespace Geometry {

class ScanBox {
public:
    ScanBox();
    ScanBox(const SegmentScan &scan);
    ScanBox(const ScanBox &sb);

    ScanBox &operator=(const ScanBox &sb);
    void augment(const Eigen::Matrix2d &covxy, double sigmaSignificance = 3);

    bool intersects(const ScanBox &sb) const;
    Quadrilateral quadrilateral() const;

    /*
     * A. Melkman, "On-line construction of the convex hull of a simple polyline",
     *  Information Processing Letters, vol. 25, pp. 11-12, 1987.
     */
    static std::deque<Point> convexHull(const SegmentScan &scan);

    friend ScanBox operator*(const Rototranslation &rt, const ScanBox &sb);
    friend LoggerStream &operator<<(LoggerStream &stream, const ScanBox &sb);

private:
    double angle;
    double minX, maxX;
    double minY, maxY;
};

}
}


#endif /* SCANBOX_H_ */
