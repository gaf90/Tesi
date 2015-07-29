#include "odometrydata.h"

namespace Data {

OdometryData::OdometryData(double timestamp, const Pose &pose) :
    timestamp(timestamp), pose(pose)
{
}

OdometryData::OdometryData(const OdometryData &odo) :
    Message(), timestamp(odo.timestamp), pose(odo.pose)
{
}

OdometryData::~OdometryData()
{
}

double OdometryData::getTimestamp() const
{
    return timestamp;
}

const Pose &OdometryData::getPose() const
{
    return pose;
}

}
