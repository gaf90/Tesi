#ifndef SENSOR_H
#define SENSOR_H

#include <QObject>
#include "data/message.h"

namespace Middleware {

/**
 * @brief Generic sensor interface
 *
 * The Sensor virtual class defines the common interface that must be
 * implemented by middleware classes that signal sensor data to the main
 * application.
 *
 * The particular sensorial data emitted (Message) is dependent on the
 * Sensor implementation, hence additional information should be
 * sought in the relative subclass documentation
 */
class Sensor : public QObject
{
    Q_OBJECT

protected:
    /**
     * Constructs a new Sensor
     *
     * @param parent Optional QObject parent
     */
    Sensor(QObject *parent = 0);

public:
    /**
     * Destroys the Sensor
     */
    virtual ~Sensor();

public slots:
    /**
     * This slot must be implemented by subclasses in order to receive messages
     * from the underlying connection controller. In case the message is of
     * interest it should be parsed and an appropriate sigSensorData signal should
     * be emitted
     *
     * @param message The message received by the underlying controller
     */
    virtual void onMessageReceived(const Data::Message &message) = 0;

signals:
    /**
     * This signal is emitted whenever new sensor data is available.
     * Refer to the particular Sensor implementation for information about the
     * actual type of Message passed
     *
     * @param data The sensor data to communicate
     */
    void sigSensorData(const Data::Message &data);
};

}

#endif // SENSOR_H
