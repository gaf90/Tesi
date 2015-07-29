#ifndef ODOMETRYSENSOR_H
#define ODOMETRYSENSOR_H

#include "sensor.h"
#include <stdio.h>

namespace Middleware {

/**
 * @brief Sensor for odometric data
 *
 * OdometrySensor implements a Sensor that parses odometric data and signals it
 * through the common Sensor interface in the form of an OdometryData message
 *
 * @see OdometryData
 */
class OdometrySensor : public Sensor
{
    Q_OBJECT

    FILE *fp;

public:
    /**
     * Constructs a new OdometrySensor
     *
     * @param parent Optional QObject parent
     */
    OdometrySensor(QObject *parent = 0);

    /**
     * Destroys the OdometrySensor
     */
    virtual ~OdometrySensor();

public slots:

    /**
     * Sensor::onMessageReceived() implementation for OdometrySensor
     */
    virtual void onMessageReceived(const Data::Message &message);
};

}

#endif // ODOMETRYSENSOR_H
