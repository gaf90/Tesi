#ifndef INSDATA_H
#define INSDATA_H

#include "message.h"
#include "pose3d.h"

namespace Data {
class INSData : public Message
{
public:
    INSData();

    INSData(double timestamp, const Pose3D &pose);

    INSData(const INSData &ins);

    virtual ~INSData();

    double getTimestamp() const;

    const Pose3D &getPose() const;

private:
    double timestamp;
    Pose3D pose;
};
}

#endif // INSDATA_H
