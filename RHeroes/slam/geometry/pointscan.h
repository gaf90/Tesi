/*
 * pointscan.h
 *
 *  Created on: 24/gen/2012
 *      Author: Mladen Mazuran
 */

#ifndef POINTSCAN_H_
#define POINTSCAN_H_

#include "data/laserdata.h"
#include "slam/geometry/point.h"
#include "slam/geometry/rototranslation.h"
#include <QString>
#include <QVector>
#include <cmath>

namespace SLAM {
namespace Geometry {

class PointScan
{
public:
    PointScan();
    PointScan(const Data::LaserData &data);
    PointScan(const PointScan &p);
    virtual ~PointScan();

    const Geometry::Point &operator[](int i) const;
    double range(int i) const;
    double angle(int i) const;
    double fov() const;
    double resolution() const;
    bool valid(int i) const;
    const QVector<double> &readings() const;
    const QVector<double> &angles() const;
    const QVector<Geometry::Point> &points() const;
    const Geometry::Point &center() const;
    int size() const;

    static PointScan medianFilter(const PointScan &ps, int windowSize = 3);

    Geometry::Point centroid() const;
    Eigen::Matrix2d sampleCovariance() const;

    friend PointScan operator*(const Geometry::Rototranslation &rt, const PointScan &scan);
    friend LoggerStream &operator<<(LoggerStream &stream, const PointScan &scan);
private:
    QVector<double> vreadings;
    QVector<double> vangles;
    QVector<Geometry::Point> vpoints;
    Geometry::Point vcenter;
    double ffov;
};

inline const Geometry::Point &PointScan::operator [](int i) const { return vpoints[i]; }
inline double PointScan::range(int i) const { return vreadings[i]; }
inline double PointScan::angle(int i) const { return vangles[i]; }
inline double PointScan::fov() const { return ffov; }
inline double PointScan::resolution() const { return ffov / (size() - 1); }
inline bool PointScan::valid(int i) const { return vreadings[i] > 0 && vreadings[i] < 20; }
inline int PointScan::size() const { return vreadings.count(); }
inline const Geometry::Point &PointScan::center() const { return vcenter; }
inline const QVector<double> &PointScan::readings() const { return vreadings; }
inline const QVector<double> &PointScan::angles() const { return vangles; }
inline const QVector<Geometry::Point> &PointScan::points() const { return vpoints; }

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* POINTSCAN_H_ */
