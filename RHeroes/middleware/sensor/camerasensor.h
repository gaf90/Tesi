#ifndef CAMERASENSOR_H
#define CAMERASENSOR_H

#include "sensor.h"

namespace Middleware {

/**
 * @brief Sensor for camera data
 *
 * CameraSensor implements a Sensor that parses raw image data obtained by the
 * UPIS controller and signals it through the common Sensor interface in the
 * form of a CameraData message
 *
 * @see CameraData
 */
class CameraSensor : public Sensor
{
    Q_OBJECT

public:
    /**
     * Constructs a new CameraSensor
     *
     * @param parent Optional QObject parent
     */
    CameraSensor(QObject *parent = 0);

    /**
     * Destroys the CameraSensor
     */
    virtual ~CameraSensor();

public slots:

    /**
     * Sensor::onMessageReceived() implementation for CameraSensor
     */
    void onMessageReceived(const Data::Message &message);
};

}

#endif // CAMERASENSOR_H
