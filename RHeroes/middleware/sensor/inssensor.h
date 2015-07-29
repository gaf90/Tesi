#ifndef INSSENSOR_H
#define INSSENSOR_H

#include <stdio.h>
#include "sensor.h"

namespace Middleware {


class INSSensor : public Sensor
{
    Q_OBJECT

    FILE *fp;
public:
    INSSensor(QObject *parent = 0);

    ~INSSensor();

public slots:

    /**
     * Sensor::onMessageReceived() implementation for INSSensor
     */
    virtual void onMessageReceived(const Data::Message &message);
};
}

#endif // INSSENSOR_H
