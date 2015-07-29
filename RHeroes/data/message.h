#ifndef MESSAGE_H
#define MESSAGE_H

#include <QString>

namespace Data {

/**
 * @brief Generic message container
 *
 * The Message class represents a common interface for message exchange used
 * for sensors, drivers, low-level controllers and so on
 *
 * @see Sensor
 * @see Driver
 */
class Message
{
public:
	virtual ~Message() {}
    /**
     * This method is meant to be overridden by subclasses, it provides a
     * string representation of the Message (or just some information about it).
     * A default empty string implementation is provided.
     * The presence of this virtual method also ensures that RTTI support is
     * compiled in for this class
     *
     * @return A string representation of the Message
     */
    virtual operator QString() { return QString(); }
};

}

#endif // MESSAGE_H
