/*
 * pointscan.cpp
 *
 *  Created on: 24/gen/2012
 *      Author: Mladen Mazuran
 */

#include "pointscan.h"

#include <QStringList>

namespace SLAM {
namespace Geometry {

using namespace Data;

PointScan::PointScan() :
    vreadings(0), vangles(0), vpoints(0), vcenter(0, 0), ffov(0)
{
}

PointScan::PointScan(const LaserData &data) :
    vreadings(data.getReadings().size()), vangles(data.getReadings().size()),
    vpoints(data.getReadings().size()), vcenter(0, 0), ffov(data.getFOV())
{
    const QList<double> &r = data.getReadings();
    int count = r.size();

    for(int i = 0; i < count; i++) {
        double theta = i * ffov / (count - 1) - 0.5 * ffov + M_PI_2;
        vreadings[i] = r[i];
        vangles[i] = theta;
        vpoints[i] = Point(vreadings[i] * std::cos(theta), vreadings[i] * std::sin(theta));
    }
}

PointScan::PointScan(const PointScan &p) :
    vreadings(p.vreadings), vangles(p.vangles), vpoints(p.vpoints), vcenter(p.vcenter), ffov(p.ffov)
{
}

PointScan::~PointScan()
{
}

Geometry::Point PointScan::centroid() const
{
    int count = 0;
    Geometry::Point sum(0,0);
    for(int i = 0; i < vpoints.size(); i++) {
        if(valid(i)) {
            sum += vpoints[i];
            count++;
        }
    }
    return sum / count;
}

Eigen::Matrix2d PointScan::sampleCovariance() const
{
    int count = 0;
    Eigen::Vector2d m = centroid().vector();
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for(int i = 0; i < vpoints.size(); i++) {
        if(valid(i)) {
            Eigen::Vector2d diff = vpoints[i].vector() - m;
            cov += diff * diff.transpose();
            count++;
        }
    }
    return cov / (count - 1);
}

PointScan PointScan::medianFilter(const PointScan &ps, int windowSize)
{
    const int wHalf = windowSize / 2;
    double *window = new double[windowSize];
    PointScan ret;
    ret.vreadings.resize(ps.size());
    ret.vpoints.resize(ps.size());
    ret.vangles = ps.vangles;
    ret.ffov = ps.ffov;
    ret.vcenter = ps.vcenter;

    for(int i = 0; i < wHalf; i++) {
        const int j = ps.size() - i - 1;
        ret.vreadings[i] = ps.vreadings[i];
        ret.vreadings[j] = ps.vreadings[j];
        ret.vpoints[i] = Point(
                ret.vreadings[i] * std::cos(ret.vangles[i]),
                ret.vreadings[i] * std::sin(ret.vangles[i]));
        ret.vpoints[j] = Point(
                ret.vreadings[j] * std::cos(ret.vangles[j]),
                ret.vreadings[j] * std::sin(ret.vangles[j]));
    }

    for(int i = wHalf; i < ps.size() - wHalf; i++) {
        QVector<double>::const_iterator pos = ps.vreadings.begin() + i;
        std::copy(pos - wHalf, pos + wHalf + 1, window);
        std::nth_element(window, window + wHalf, window + windowSize);
        ret.vreadings[i] = window[wHalf];
        ret.vpoints[i] = Point(
                ret.vreadings[i] * std::cos(ret.vangles[i]),
                ret.vreadings[i] * std::sin(ret.vangles[i]));
    }

    delete window;
    return ret;
}


PointScan operator*(const Rototranslation &rt, const PointScan &scan)
{
    PointScan rototranslated;
    rototranslated.vreadings = scan.vreadings;
    rototranslated.vangles = scan.vangles;
    rototranslated.vpoints = QVector<Point>(scan.vpoints.size());
    rototranslated.ffov = scan.ffov;
    for(int i = 0; i < scan.size(); i++) {
        rototranslated.vpoints[i] = rt * scan.vpoints[i];
    }
    rototranslated.vcenter = rt * scan.vcenter;
    return rototranslated;
}

LoggerStream &operator<<(LoggerStream &stream, const PointScan &scan)
{
    return stream << scan.vpoints;
}

} /* namespace Geometry */
} /* namespace SLAM */
