#ifndef DRIVER_H
#define DRIVER_H

#include <QObject>
#include "data/message.h"

namespace Middleware {

/**
 * @brief Generic driver interface
 *
 * The Sensor virtual class defines the common interface that must be
 * implemented by middleware classes that communicate commands to an underlying
 * controller.
 *
 * The particular command (Message) expected is dependent on the Driver
 * implementation, hence additional information should be sought in the
 * relative subclass documentation
 */
class Driver : public QObject
{
    Q_OBJECT

protected:
    /**
     * Constructs a new Driver
     *
     * @param parent Optional QObject parent
     */
    Driver(QObject *parent = 0);

public:
    /**
     * Destroys the Driver
     */
    virtual ~Driver();

public slots:
    /**
     * This slot must be implemented by subclasses in order to receive messages
     * from the driver user (i.e. RobotController). In case the message is of
     * interest it should be interpreted and an appropriate sigDriveMessageSend
     * signal should be emitted for the underlying controller.
     *
     * Refer to the particular Driver implementation for information about the
     * actual type of Message expected
     *
     * @param message The message sent by the driver user
     */
    virtual void onDriverMessage(const Data::Message &message) = 0;

signals:
    /**
     * This signal is emitted whenever the driver deems necessary to communicate
     * a command to the underlying controller.
     *
     * @param data The command message aimed at the underlying controller
     */
    void sigDriveMessageSend(const Data::Message &data);
};

}

#endif // DRIVER_H
