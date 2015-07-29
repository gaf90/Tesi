/*
 * slamengine.h
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef SLAMENGINE_H_
#define SLAMENGINE_H_

#include "slam/geometry/pointscan.h"
#include "slam/map.h"
#include "data/pose.h"
#include "data/insdata.h"
#include <QObject>

namespace SLAM {
namespace Engine {

class SLAMEngine : public QObject
{
    Q_OBJECT

public:
    virtual ~SLAMEngine() {}
    virtual Map getMap() const = 0;
    virtual void handleScan(double timestamp, const Geometry::PointScan &scan) = 0;
    virtual void handleOdometry(double timestamp, const Data::Pose &pose) = 0;
    virtual void handleINS(const Data::INSData &ins) = 0;

signals:
    void newRobotPose(TimedPose pose);

};

} /* namespace Engine */
} /* namespace SLAM */

#endif /* SLAMENGINE_H_ */


