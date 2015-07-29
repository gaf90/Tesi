#ifndef UPISCONTROLLER_H
#define UPISCONTROLLER_H

#include "abstractsocketcontroller.h"
#include "data/message.h"
#include "shared/constants.h"
#include <QByteArray>
#include <QTimer>
#include <stdint.h>

namespace Connection {

/**
 * @brief UPIS connection controller
 *
 * UPISController manages the low-level communication to the UPIS server.
 * A portion of the UDK draw area is periodically requested to the server and
 * the raw image data is emitted in the form of a UPISMessage
 *
 * @see UPISMessage
 */
class UPISController : public AbstractSocketController
{
    Q_OBJECT

public:
    /**
     * Constructs a new UPISController. Two mandatory pieces of information
     * are necessary: how frequently to request frames to UPIS (in frames per
     * second), and the incremental number identifying the robot. The latter
     * is necessary in order to determine what portion of the UDK draw area to
     * request.
     *
     * @param fps The frame frequency requested
     * @param nRobot Incremental number identifying the robot
     * @param parent Optional QObject parent
     */
    explicit UPISController(double fps, int nRobot, QObject *parent = 0);

    /**
     * Destroys the UPISController
     */
    virtual ~UPISController();

    /**
     * This method allows to dinamically change the framerate.
     *
     * @param fps double containing the number of frames per second.
     */
    void setFPS(double fps);

signals:
    /**
     * Signals a Message to the connected Sensor(s). The actual object provided
     * is an instance of UPISMessage containing raw image data; the definition
     * of the signal reports a generic Message in order to be compatible with
     * the slot definition of the Sensor interface
     *
     * @param msg Message containing raw image data
     */
    void sigMessage(const Data::Message &msg);

public slots:
    /**
     * @copydoc AbstractSocketController::connectToHost()
     *
     * Overridden in order to start frame request timer
     */
    void connectToHost(const QString &address, quint16 port);

    /**
     * @copydoc AbstractSocketController::disconnectFromHost()
     *
     * Overridden in order to stop frame request timer
     */
    void disconnectFromHost();

protected slots:
    /**
     * AbstractSocketController::invokeReadData() implementation for
     * UPISController
     */
    void invokeReadData();

private slots:
    void requestFrame();

private:
    QByteArray rawData;
    QTimer timer;
    double fps;
    bool reading, jpeg;
    uint32_t x, y, sizeLeft;
};

}

#endif // UPISCONTROLLER_H
