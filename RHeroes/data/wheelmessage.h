#ifndef WHEELMESSAGE_H
#define WHEELMESSAGE_H

#include "message.h"
#include "serializable.h"

namespace Data {

/**
 * @brief Wheel control message
 *
 * WheelMessage is used as a command exchange parameter with WheelDriver, it
 * is a simple container for left and right wheel speed
 *
 * @see WheelDriver
 */
class WheelMessage : public Message, public Serializable
{
public:

    WheelMessage();

    /**
     * Initialises a new WheelMessage with the requested wheel speeds. Both
     * inputs are expressed as a fraction 0 <= q <= 1 of the maximum velocity
     * of the wheels (TODO: check this)
     *
     * @param vleft Left wheel speed
     * @param vright Right wheel speed
     */
    WheelMessage(double vleft, double vright);

    /**
     * Destroys the WheelMessage
     */
    virtual ~WheelMessage();

    /**
     * @return The left wheel speed of the robot
     */
    double getLeftWheelSpeed() const;

    /**
     * @return The right wheel speed of the robot
     */
    double getRightWheelSpeed() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    double vleft, vright;

};

}

#endif // WHEELMESSAGE_H
