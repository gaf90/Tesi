#include "inssensor.h"
#include "data/insdata.h"
#include "data/usarmessage.h"
#include <typeinfo>
#include <QStringList>

namespace Middleware {
INSSensor::INSSensor(QObject *parent) :
    Sensor(parent)
{
}

INSSensor::~INSSensor()
{
}

void INSSensor::onMessageReceived(const Data::Message &message)
{
    if(typeid(message) == typeid(const Data::USARMessage &)) {
        const Data::USARMessage &msg = (const Data::USARMessage &) message;

        if(msg.getType() == "SEN" && msg.contains("Type") &&
                msg["Type"] == "INS") {
            QString location = msg["Location"];
            QString orientation = msg["Orientation"];
            if(location.length() > 0 && orientation.length() > 0){
                QStringList list = location.split(",");

                double timestamp = msg["Time"].toDouble();

                /* Get the doubles associated to x, y, theta */
                double ins_x = list[0].toDouble();
                double ins_y = list[1].toDouble();
                double ins_z = list[2].toDouble();

                list = orientation.split(",");
                double orientation_r = list[0].toDouble();
                double orientation_p = list[1].toDouble();
                double orientation_y = list[2].toDouble();

                emit sigSensorData(Data::INSData(timestamp,
                     Data::Pose3D(ins_y, ins_x, - ins_z, orientation_r, orientation_p, - orientation_y)));
            }
        }
    }
}
}
