#ifndef AIRDRIVEMESSAGE_H
#define AIRDRIVEMESSAGE_H

#include "message.h"
#include "serializable.h"

namespace Data {

/**
 * @brief air driving control message
 *
 * AirDriveMessage is used as a command exchange parameter with AirControlDriver, it
 * is a simple container for altitude, linear, rotational and lateral speeds
 *
 * @see AirControlDriver
 */
class AirDriveMessage: public Message, public Serializable
{
public:
    AirDriveMessage();

    /**
     * Initialises a new AirDriveMessage with the requested wheel speeds.
     */
    AirDriveMessage(double altitudeSpeed, double linearSpeed, double lateralSpeed, double rotationalSpeed);

    /**
     * Destroys the AirDriveMessage
     */
    virtual ~AirDriveMessage();

    double getAltitudeSpeed() const;
    double getLinearSpeed() const;
    double getLateralSpeed() const;
    double getRotationalSpeed() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    double altitudeSpeed, linearSpeed, lateralSpeed, rotationalSpeed;
};

}

#endif // AIRDRIVEMESSAGE_H
