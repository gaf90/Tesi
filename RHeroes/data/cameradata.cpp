#include "cameradata.h"
#include "shared/constants.h"
#include <QBuffer>

namespace Data {

CameraData::CameraData() :
    loadedFrame(), frame(loadedFrame)
{
}

CameraData::CameraData(const QImage &frame) :
    loadedFrame(), frame(frame)
{
}

CameraData::~CameraData()
{
}

const QImage &CameraData::getFrame() const
{
    return frame;
}

void CameraData::serializeTo(QDataStream &stream) const
{
    QByteArray data;
    QBuffer buffer(&data);
    buffer.open(QIODevice::WriteOnly);
    frame.save(&buffer, WS_CAMERA_FORMAT, WS_CAMERA_QUALITY);

    stream << data;
}

void CameraData::deserializeFrom(QDataStream &stream)
{
    QByteArray data;
    stream >> data;

    loadedFrame.loadFromData(data, WS_CAMERA_FORMAT);
}

}
