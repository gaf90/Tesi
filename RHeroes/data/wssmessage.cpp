#include "wssmessage.h"

namespace Data {

const QString WSSMessage::TYPE_ROBOT_DATA = "R";

WSSMessage::WSSMessage() :
    USARMessage(), reliableDelivering(false)
{
}

WSSMessage::WSSMessage(const QString &line) :
    USARMessage(line), reliableDelivering(false)
{
}

WSSMessage::WSSMessage(const WSSMessage &message) :
    USARMessage(message), reliableDelivering(false)
{
}


WSSMessage::~WSSMessage()
{
}

void WSSMessage::setRobotData(
    const QString &peer, const QByteArray *robotData)
{
    this->setType(WSSMessage::TYPE_ROBOT_DATA);
    this->peer = peer;
    this->robotData = robotData;
}

const QString &WSSMessage::getPeer() const
{
    return peer;
}

const QByteArray *WSSMessage::getRobotData() const
{
    return robotData;
}

bool WSSMessage::mustBeDelivered() const
{
    return reliableDelivering;
}

void WSSMessage::setDeliveringMode(bool reliable)
{
    reliableDelivering = reliable;
}

}
