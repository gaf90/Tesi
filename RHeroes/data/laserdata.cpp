#include "laserdata.h"

namespace Data {

LaserData::LaserData(
        double timestamp, double fov, double resolution,
        const QList<double> &readings) :
    timestamp(timestamp), fov(fov), resolution(resolution), readings(readings)
{
}

LaserData::~LaserData()
{
}

double LaserData::getTimestamp() const
{
    return timestamp;
}

const QList<double> &LaserData::getReadings() const
{
    return readings;
}

double LaserData::getFOV() const
{
    return fov;
}

double LaserData::getResolution() const
{
    return resolution;
}

}
