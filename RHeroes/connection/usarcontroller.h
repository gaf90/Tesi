#ifndef USARCONTROLLER_H
#define USARCONTROLLER_H

#include "abstractsocketcontroller.h"
#include "data/message.h"

namespace Connection {

/**
 * @brief USARSim connection controller
 *
 * USARController manages the low-level communication to the USARSim server.
 * A signal is emitted every time a message is received from the server,
 * conversely the controller will forward a message to the server in case a
 * connected Driver has requested so. Both input and output communication data
 * is encapsulated in USARMessage instances
 *
 * @see USARMessage
 */
class USARController : public AbstractSocketController
{
    Q_OBJECT

public:
    /**
     * Constructs a new USARController
     *
     * @param parent Optional QObject parent
     */
    explicit USARController(QObject *parent = 0);

    /**
     * Destroys the USARController
     */
    virtual ~USARController();

signals:
    /**
     * Signals a Message to the connected Sensor(s). The actual object provided
     * is an instance of USARMessage containing the information sent by the
     * server; the definition of the signal reports a generic Message in order
     * to be compatible with thecorresponding  slot definition of the Sensor
     * interface
     *
     * @param msg Message containing information sent by USARSim
     */
    void sigMessage(const Data::Message &msg);

public slots:
    /**
     * Slot aimed at sending a message to the USARSim message. This method
     * expects a USARMessage as input argument, any different object will be
     * quietly ignored. The definition of the slot reports a generic Message in
     * order to be compatible with the corresponding signal definition of the
     * Driver interface
     *
     * @param msg Message containing information to send to USARSim
     */
    void sendMessage(const Data::Message &msg);

protected slots:
    /**
     * AbstractSocketController::invokeReadData() implementation for
     * USARController
     */
    void invokeReadData();
};

}

#endif // USARCONTROLLER_H
