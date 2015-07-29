#include "camerasensor.h"

#include <typeinfo>
#include <QImage>
#include "data/upismessage.h"
#include "data/cameradata.h"
#include <stdint.h>

namespace Middleware {

using namespace Data;

CameraSensor::CameraSensor(QObject *parent) :
    Sensor(parent)
{
}

CameraSensor::~CameraSensor()
{
}

void CameraSensor::onMessageReceived(const Message &message)
{
    if(typeid(message) == typeid(const UPISMessage &))
    {
        const UPISMessage &data = (const UPISMessage &) message;
        if(data.isJPEG())
        {
            emit sigSensorData(CameraData(QImage::fromData(data)));
        }
        else
        {
            uint16_t w, h;
            QByteArray header = data.left(4);
            QDataStream stream(&header, QIODevice::ReadOnly);
            stream.setByteOrder(QDataStream::BigEndian);
            stream >> w >> h;

            QImage img(((const uchar *) data.data()) + 4, w, h, w*3, QImage::Format_RGB888);
            emit sigSensorData(CameraData(img.rgbSwapped()));
        }
    }
}

}
