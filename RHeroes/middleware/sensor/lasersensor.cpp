#include "lasersensor.h"
#include "data/laserdata.h"
#include "data/usarmessage.h"
#include <typeinfo>
#include <QStringList>

namespace Middleware {

using namespace Data;

LaserSensor::LaserSensor(QObject *parent) :
    Sensor(parent)
{
}

LaserSensor::~LaserSensor()
{
}

void LaserSensor::onMessageReceived(const Message &message) {
    if(typeid(message) == typeid(const USARMessage &)) {
        const USARMessage &msg = (const USARMessage &) message;

        if(msg.getType() == "SEN" &&
                msg.contains("Type") &&
                (msg["Type"] == "RangeScanner" || msg["Type"] == "IRScanner")) {

            QList<double> readings;
            double timestamp = msg["Time"].toDouble();
            double fov = msg["FOV"].toDouble();
            double resolution = msg["Resolution"].toDouble();

            QStringList strReadings = msg["Range"].split(",");
            foreach(QString num, strReadings) {
                readings.append(num.toDouble());
            }

            if(strReadings.size() > 1) {
                emit sigSensorData(LaserData(timestamp, fov, resolution, readings));
            }
        }
    }
}

}
