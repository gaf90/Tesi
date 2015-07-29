#ifndef AIRCONTROLDRIVER_H
#define AIRCONTROLDRIVER_H

#include "driver.h"

namespace Middleware {

/**
 * @brief Wheel motion driver for the P3AT
 *
 * WheelDriver implements a wheel motion driver for the P3AT. The driver expects
 * WheelMessage objects as communication medium
 *
 * @see WheelMessage
 */
class AirControlDriver : public Driver
{
    Q_OBJECT

public:

    /**
     * Constructs a new AirControlDriver
     *
     * @param parent Optional QObject parent
     */
    AirControlDriver(QObject *parent = 0);

    /**
     * Destroys the AirControlDriver
     */
    virtual ~AirControlDriver();

public slots:
    /**
     * Driver::onDriverMessage() implementation for AirControlDriver
     */
    void onDriverMessage(const Data::Message &message);
};

}
#endif // AIRCONTROLDRIVER_H
