#ifndef SENSORDRIVER_H
#define SENSORDRIVER_H

#include "driver.h"

namespace Middleware {

/**
 * @brief Generic driver interface with sensorial capabilities
 *
 * Driver interface with sensorial capabilities, needed because diamond
 * inheritance with QObject as base class is unsupported by moc
 *
 * @see Sensor
 */
class SensorDriver : public Driver
{
    Q_OBJECT

protected:
    /**
     * Constructs a new SensorDriver
     *
     * @param parent Optional QObject parent
     */
    SensorDriver(QObject *parent = 0);

public:
    /**
     * Destroys the SensorDriver
     */
    virtual ~SensorDriver();

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

#endif // SENSORDRIVER_H
