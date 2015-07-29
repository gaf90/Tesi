#include "wheelmessage.h"

namespace Data {

WheelMessage::WheelMessage() :
    vleft(0), vright(0)
{
}

WheelMessage::WheelMessage(double vleft, double vright) :
    vleft(vleft), vright(vright)
{
}

WheelMessage::~WheelMessage()
{
}

double WheelMessage::getLeftWheelSpeed() const
{
    return vleft;
}

double WheelMessage::getRightWheelSpeed() const
{
    return vright;
}

void WheelMessage::serializeTo(QDataStream &stream) const
{
    stream << vleft << vright;
}

void WheelMessage::deserializeFrom(QDataStream &stream)
{
    stream >> vleft >> vright;
}

}
