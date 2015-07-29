#ifndef STATESENSOR_H
#define STATESENSOR_H

#include <QObject>
#include "sensor.h"
namespace Middleware{
class StateSensor : public Sensor
{
    Q_OBJECT
public:
    explicit StateSensor(QObject *parent = 0);
    virtual ~StateSensor();
signals:
    
public slots:
    /**
     * Sensor::onMessageReceived() implementation for LaserSensor
     */
    virtual void onMessageReceived(const Data::Message &message);
};
}

#endif // STATESENSOR_H
