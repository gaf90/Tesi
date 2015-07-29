#ifndef GROUNDTRUTHSENSOR_H
#define GROUNDTRUTHSENSOR_H

#include "sensor.h"

namespace Middleware {

class GroundTruthSensor : public Sensor
{
    Q_OBJECT
public:
    explicit GroundTruthSensor(QObject *parent = 0);

    virtual ~GroundTruthSensor();
    
public slots:
    virtual void onMessageReceived(const Data::Message &message);
};

}

#endif // GROUNDTRUTHSENSOR_H
