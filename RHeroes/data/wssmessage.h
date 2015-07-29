#ifndef WSSMESSAGE_H
#define WSSMESSAGE_H

#include "usarmessage.h"
#include <QByteArray>

namespace Data {

/**
 * @brief Message representing data sent to and from WSS
 *
 * This class is functionally equivalent to USARMessage, just redefined for
 * WSSController, refer to USARMessage for more information
 *
 * @see USARController
 * @see USARMessage
 */
class WSSMessage : public USARMessage
{
public:
    /**
     * Initialises an empty WSSMessage
     */
    WSSMessage();

    /**
     * Initialises an WSSMessage from a string serialised representation of
     * a WSS message, the input is parsed and the type and key-value pairs
     * are filled
     *
     * @param line The input string serialised WSS message
     */
    explicit WSSMessage(const QString &line);

    /**
     * Initialises an WSSMessage from another existing one
     *
     * @param message The WSSMessage to clone
     */
    WSSMessage(const WSSMessage &message);

    /**
     * Destroys the WSSMessage
     */
    virtual ~WSSMessage();

    /**
     * Instructs to send data to the specified robot
     *
     * @param destination Destination robot
     * @param robotData Raw data to send
     */
    void setRobotData(
        const QString &peer, const QByteArray *robotData);

    const QString &getPeer() const;
    const QByteArray *getRobotData() const;

    bool mustBeDelivered() const;
    void setDeliveringMode(bool reliable);

    static const QString TYPE_ROBOT_DATA;

private:
    const QByteArray *robotData;
    QString peer;
    bool reliableDelivering;
};

}

#endif // WSSMESSAGE_H
