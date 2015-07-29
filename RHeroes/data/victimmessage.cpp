#include "victimmessage.h"
#include "shared/constants.h"
#include <QBuffer>
namespace Data{

VictimMessage::VictimMessage():
    confidence(0),
    position(Pose()),
    robotName("unknown"),
    img(loadedFrame),
    loadedFrame()
{
}

VictimMessage::VictimMessage(QString robotName, QImage image, double confidence, Pose position):
    confidence(confidence),
    position(position),
    robotName(robotName),
    img(image),
    loadedFrame()
{
}

VictimMessage::~VictimMessage()
{
}

void VictimMessage::serializeTo(QDataStream &stream) const
{
    QByteArray data;
    QBuffer buffer(&data);
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, WS_CAMERA_FORMAT, WS_CAMERA_QUALITY);

    stream << robotName << position << confidence << data;
}

void VictimMessage::deserializeFrom(QDataStream &stream)
{
    QByteArray data;
    stream >> robotName >> position >> confidence >> data;
    loadedFrame.loadFromData(data, WS_CAMERA_FORMAT);
}

double VictimMessage::getConfidence() const
{
    return this->confidence;
}

Pose VictimMessage::getPosition() const
{
    return this->position;
}

QString VictimMessage::getRobotName() const
{
    return this->robotName;
}

const QImage &VictimMessage::getVictimImage() const
{
    return this->img;
}
}
