/*
 * scanbox.cpp
 *
 *  Created on: 11/gen/2013
 *      Author: Mladen Mazuran
 */

#include "scanbox.h"
#include "shared/utilities.h"
#include <Eigen/Eigenvalues>

namespace SLAM {
namespace Geometry {

ScanBox::ScanBox() : angle(0), minX(INFINITY), maxX(INFINITY), minY(INFINITY), maxY(INFINITY)
{
}

ScanBox::ScanBox(const SegmentScan &scan) :
        minX(0), maxX(0), minY(0), maxY(0)
{
    const std::deque<Point> v = convexHull(scan);

    /*
     * Simple O(||v||^2) algorithm, maybe do it with rotating calipers in the future, see:
     * Toussaint, Godfried T. "Solving geometric problems with the rotating calipers",
     *  Proc. MELECON '83, Athens, 1983.
     */
    double minArea = INFINITY;

    for(size_t i = 0; i < v.size() - 1; i++) {
        const Point d = v[i + 1] - v[i];
        const double angle = std::atan2(d.y(), d.x());
        const Rototranslation r(-angle);

        double xmin = INFINITY, xmax = -INFINITY;
        double ymin = INFINITY, ymax = -INFINITY;

        for(size_t j = 0; j < v.size() - 1; j++) {
            const Point q = r * v[j];
            xmin = min(xmin, q.x());
            xmax = max(xmax, q.x());
            ymin = min(ymin, q.y());
            ymax = max(ymax, q.y());
        }

        const double area = (xmax - xmin) * (ymax - ymin);
        if(area < minArea) {
            minArea = area;
            this->angle = angle;
            minX = xmin;
            maxX = xmax;
            minY = ymin;
            maxY = ymax;
        }
    }
}

ScanBox::ScanBox(const ScanBox &sb) :
        angle(sb.angle), minX(sb.minX), maxX(sb.maxX), minY(sb.minY), maxY(sb.maxY)
{
}

ScanBox &ScanBox::operator=(const ScanBox &sb)
{
    angle = sb.angle;
    minX = sb.minX;
    maxX = sb.maxX;
    minY = sb.minY;
    maxY = sb.maxY;
    return *this;
}

void ScanBox::augment(const Eigen::Matrix2d &covxy, double sigmaSignificance)
{
    Eigen::Matrix2d R, cov;
    R <<
            std::cos(angle), std::sin(angle),
           -std::sin(angle), std::cos(angle);
    cov = R * covxy * R.transpose();
    minX -= sigmaSignificance * std::sqrt(cov(0,0));
    maxX += sigmaSignificance * std::sqrt(cov(0,0));
    minY -= sigmaSignificance * std::sqrt(cov(1,1));
    maxY += sigmaSignificance * std::sqrt(cov(1,1));
}

bool ScanBox::intersects(const ScanBox &sb) const
{
    const Quadrilateral q1 = quadrilateral();
    const Quadrilateral q2 = sb.quadrilateral();

    if(q1.contains(q2.p1()) || q1.contains(q2.p2()) || q1.contains(q2.p3()) ||
            q1.contains(q2.p4()) || q2.contains(q1.p1()) || q2.contains(q1.p2()) ||
            q2.contains(q1.p3()) || q2.contains(q1.p4())) {
        return true;
    }

    const LineSegment ls1[4] = {
            LineSegment(q1.p1(), q1.p2()),
            LineSegment(q1.p2(), q1.p3()),
            LineSegment(q1.p3(), q1.p4()),
            LineSegment(q1.p4(), q1.p1())
    };
    const LineSegment ls2[4] = {
            LineSegment(q2.p1(), q2.p2()),
            LineSegment(q2.p2(), q2.p3()),
            LineSegment(q2.p3(), q2.p4()),
            LineSegment(q2.p4(), q2.p1())
    };

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(ls1[i].intersects(ls2[j])) {
                return true;
            }
        }
    }

    return false;
}

Quadrilateral ScanBox::quadrilateral() const
{
    const Rototranslation rt(angle);
    const Point p1(minX, minY), p2(minX, maxY), p3(maxX, maxY), p4(maxX, minY);
    return Quadrilateral(rt * p1, rt * p2, rt * p3, rt * p4);
}

/* Returns true if q is at the left of oriented line segment (p1,p2), false otherwise */
static inline bool left(const Point &p1, const Point &p2, const Point &q)
{
    Point v1 = p2 - p1, v2 = q - p1;
    return v2.x() * v1.y() - v2.y() * v1.x() < 0;
}

/*
 * A. Melkman, "On-line construction of the convex hull of a simple polyline",
 *  Information Processing Letters, vol. 25, pp. 11-12, 1987.
 */
std::deque<Point> ScanBox::convexHull(const SegmentScan &scan)
{
    std::deque<Point> chull;
    QList<Point> v = scan.toPolygon().vertices();
    if(v.size() < 3) {
        for(int i = 0; i < min(2, v.size()); i++) {
            chull.push_back(v[i]);
            return chull;
        }
    }

    chull.push_back(v[2]);
    if(left(v[0], v[1], v[2])) {
        chull.push_back(v[0]);
        chull.push_back(v[1]);
    } else {
        chull.push_back(v[1]);
        chull.push_back(v[0]);
    }
    chull.push_back(v[2]);

    for(int i = 3; i < v.size(); i++) {
        int s = chull.size();
        while(left(chull[s - 2], chull[s - 1], v[i]) &&
                left(chull[0], chull[1], v[i])) {
            i++;
            if(i == v.size()) {
                //chull.pop_back();
                return chull;
            }
        }
        while(!left(chull[s - 2], chull[s - 1], v[i])) {
            chull.pop_back();
            s--;
        }
        chull.push_back(v[i]);
        while(!left(v[i], chull[0], chull[1])) {
            chull.pop_front();
            s--;
        }
        chull.push_front(v[i]);
    }

    //chull.pop_back();
    return chull;
}

ScanBox operator*(const Rototranslation &rt, const ScanBox &sb)
{
    ScanBox newsb;
    newsb.angle = wrap(sb.angle + rt.angle());
    const Rototranslation R(-newsb.angle);
    const Point map = R * Point(rt.translation());
    newsb.minX = sb.minX + map.x();
    newsb.maxX = sb.maxX + map.x();
    newsb.minY = sb.minY + map.y();
    newsb.maxY = sb.maxY + map.y();
    return newsb;
}

LoggerStream &operator<<(LoggerStream &stream, const ScanBox &sb)
{
    return stream << sb.quadrilateral();
}

} /* namespace SLAM */
} /* namespace Geometry */



