#ifndef WHEELDRIVER_H
#define WHEELDRIVER_H

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
class WheelDriver : public Driver
{
    Q_OBJECT

public:
    /**
     * Constructs a new WheelDriver
     *
     * @param parent Optional QObject parent
     */
    WheelDriver(QObject *parent = 0);

    /**
     * Destroys the WheelDriver
     */
    virtual ~WheelDriver();

public slots:
    /**
     * Driver::onDriverMessage() implementation for WheelDriver
     */
    void onDriverMessage(const Data::Message &message);
};

}

#endif // WHEELDRIVER_H
