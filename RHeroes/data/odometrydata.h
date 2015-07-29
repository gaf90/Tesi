#ifndef ODOMETRYDATA_H
#define ODOMETRYDATA_H

#include "message.h"
#include "pose.h"

namespace Data {

/**
 * @brief Odometric data message
 *
 * This class represents odometric data readings, signalled by OdometrySensor
 *
 * @see OdometrySensor
 */
class OdometryData : public Message
{
public:
    /**
     * Constructs a new OdometryData message
     *
     * @param timestamp Timestamp of when the odometry was read
     * @param pose The pose as reported by the odometric sensor
     */
    OdometryData(double timestamp, const Pose &pose);

    /**
     * Copy constructor
     */
    OdometryData(const OdometryData &odo);

    /**
     * Destroys the OdometryData
     */
    virtual ~OdometryData();

    /**
     * @return The timestamp of when the odometry was read
     */
    double getTimestamp() const;

    /**
     * @return The pose as reported by the odometric sensor
     */
    const Pose &getPose() const;

private:
    double timestamp;
    const Pose pose;
};

}

#endif // ODOMETRYDATA_H
