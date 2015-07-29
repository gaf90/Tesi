#include "statesensor.h"
#include "data/usarmessage.h"
#include "data/statedata.h"
#include <typeinfo>

namespace Middleware{

    using namespace Data;

    StateSensor::StateSensor(QObject *parent) :
        Sensor(parent)
    {
    }
    StateSensor::~StateSensor(){

    }

    void StateSensor::onMessageReceived(const Data::Message &message)
    {
        if(typeid(message) == typeid(const USARMessage &)) {
            const USARMessage &msg = (const USARMessage &) message;
            if(msg.getType() == "STA") {

                bool light;
                if(msg["LightToggle"] == "True")
                    light = false;
                else
                    light = true;
                emit sigSensorData(StateData(msg["Type"], msg["Time"].toFloat(),
                                          msg["RearSteer"].toFloat(), msg["FrontSteer"].toFloat(), light,
                                          msg["LightIntensity"].toInt(), msg["Battery"].toInt()));
            }
        }
    }
}
