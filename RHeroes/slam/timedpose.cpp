/*
 * timedpose.cpp
 *
 *  Created on: 11/mar/2012
 *      Author: Mladen Mazuran
 */

#include "timedpose.h"

namespace SLAM {

using namespace Data;

TimedPose::TimedPose() :
    Pose(), ts(0)
{
}

TimedPose::TimedPose(double ts, const Pose &pose) :
    Pose(pose), ts(ts)
{
}

TimedPose::TimedPose(double ts, const Eigen::Vector3d &vec) :
    Pose(vec), ts(ts)
{
}


TimedPose::TimedPose(const TimedPose &pose) :
    Pose(pose), ts(pose.ts)
{
}

TimedPose::~TimedPose()
{
}

double TimedPose::getTimestamp() const
{
    return ts;
}

TimedPose &TimedPose::operator=(const TimedPose &pose)
{
	this->Pose::operator=(pose);
	ts = pose.ts;
	return *this;
}

LoggerStream &operator<<(LoggerStream &stream, const TimedPose *pose)
{
    return stream << *pose;
}

} /* namespace SLAM */
