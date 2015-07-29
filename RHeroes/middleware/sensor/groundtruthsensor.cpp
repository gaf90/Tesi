#include "groundtruthsensor.h"
#include "data/usarmessage.h"
#include "data/odometrydata.h"
#include <typeinfo>
#include <QStringList>

namespace Middleware {

using namespace Data;

GroundTruthSensor::GroundTruthSensor(QObject *parent) :
    Sensor(parent)
{
}

GroundTruthSensor::~GroundTruthSensor()
{
}

void GroundTruthSensor::onMessageReceived(const Message &message) {
    if(typeid(message) == typeid(const USARMessage &)) {
        const USARMessage &msg = (const USARMessage &) message;
        if(msg.getType() == "SEN" && msg.contains("Type") &&
                msg["Type"] == "GroundTruth") {

            /* Get values from Ground truth "sensor" */
            QString groundTruthLocation = msg["Location"];
            QString groundTruthOrientation = msg["Orientation"];
            if(groundTruthLocation.length() > 0 &&
                    groundTruthOrientation.length() > 0){
                QStringList listL = groundTruthLocation.split(",");
                QStringList listO = groundTruthOrientation.split(",");

                double timestamp = msg["Time"].toDouble();

                /* Get the doubles associated to x, y, and yaw; */
                double gt_x = listL[0].toDouble();
                double gt_y = listL[1].toDouble();
                double gt_t = listO[2].toDouble();

                /* Change reference frame (exchange x, y and take the
                   inverse angle) */
                emit sigSensorData(OdometryData(timestamp, Pose(gt_y, gt_x, - gt_t)));
            }
        }
    }
}


}
