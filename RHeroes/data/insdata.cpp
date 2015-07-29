#include "insdata.h"

namespace Data{

INSData::INSData() :
    Message()
{
}

INSData::INSData(double timestamp, const Pose3D &pose) :
    Message(),timestamp(timestamp), pose(pose)
{    
}

INSData::INSData(const INSData &ins) :
    Message(), timestamp(ins.timestamp), pose(ins.pose)
{
}

INSData::~INSData(){
}

double INSData::getTimestamp() const
{
    return timestamp;
}

const Pose3D &INSData::getPose() const
{
    return pose;
}
}
