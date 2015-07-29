#include "odometrysensor.h"
#include "data/usarmessage.h"
#include "data/odometrydata.h"
#include <typeinfo>
#include <QStringList>

//#define USING_ODOMETRY_SENSOR

namespace Middleware {

using namespace Data;

OdometrySensor::OdometrySensor(QObject *parent) :
    Sensor(parent)
{
    //fp = fopen("C:\\Users\\airlab\\Desktop\\dataset.txt", "w");
}

OdometrySensor::~OdometrySensor()
{
}

void OdometrySensor::onMessageReceived(const Message &message) {
    if(typeid(message) == typeid(const USARMessage &)) {
        const USARMessage &msg = (const USARMessage &) message;

        //fprintf(fp, QString(msg).toLatin1().data());

#ifdef USING_ODOMETRY_SENSOR
        if(msg.getType() == "SEN" && msg.contains("Type") &&
                msg["Type"] == "Odometry") {

            QString odometryPose = msg["Pose"];
            if(odometryPose.length() > 0){
                QStringList list = odometryPose.split(",");

                double timestamp = msg["Time"].toDouble();

                /* Get the doubles associated to x, y, theta */
                double odo_x = list[0].toDouble();
                double odo_y = list[1].toDouble();
                double odo_t = list[2].toDouble();

                /* Change reference frame (exchange x, y and take the
                   inverse angle) */
                emit sigSensorData(OdometryData(timestamp, Pose(odo_y, odo_x, - odo_t)));
            }
        }
#else
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
                //double ins_t = list[2].toDouble();

                list = orientation.split(",");
                //double orientation_r = list[0].toDouble();
                //double orientation_p = list[1].toDouble();
                double orientation_y = list[2].toDouble();
                emit sigSensorData(OdometryData(timestamp, Pose(ins_y, ins_x, - orientation_y)));
            }
        }
#endif

    }
}

}
