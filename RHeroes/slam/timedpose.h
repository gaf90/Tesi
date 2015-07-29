/*
 * timedpose.h
 *
 *  Created on: 11/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef TIMEDPOSE_H_
#define TIMEDPOSE_H_

#include "data/pose.h"

namespace SLAM {

class TimedPose : public Data::Pose
{
public:
    TimedPose();
    TimedPose(double ts, const Data::Pose &pose);
    TimedPose(double ts, const Eigen::Vector3d &vec);
    TimedPose(const TimedPose &pose);
    virtual ~TimedPose();

    double getTimestamp() const;
    inline double timestamp() const { return ts; }

    TimedPose &operator=(const TimedPose &pose);
    friend LoggerStream &operator<<(LoggerStream &stream, const TimedPose *pose);

private:
    double ts;
};

} /* namespace SLAM */

#endif /* TIMEDPOSE_H_ */
