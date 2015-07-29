#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include "sensor.h"

namespace Middleware {

/**
 * @brief Sensor for laser scans
 *
 * LaserSensor implements a Sensor that parses laser data and signals it
 * through the common Sensor interface in the form of LaserData message
 *
 * @see LaserData
 */
class LaserSensor : public Sensor
{
public:
    /**
     * Constructs a new LaserSensor
     *
     * @param parent Optional QObject parent
     */
    LaserSensor(QObject *parent = 0);

    /**
     * Destroys the LaserSensor
     */
    virtual ~LaserSensor();

public slots:

    /**
     * Sensor::onMessageReceived() implementation for LaserSensor
     */
    virtual void onMessageReceived(const Data::Message &message);
};

}

#endif // LASERSENSOR_H
