#include "destinationmessage.h"

namespace Data{
DestinationMessage::DestinationMessage()
{
}

DestinationMessage::DestinationMessage(int aRobotId, SLAM::Geometry::Point aPoint)
{
    robotId = aRobotId;
    point = aPoint;
}

DestinationMessage::~DestinationMessage()
{
}

int DestinationMessage::getRobotId() const
{
    return robotId;
}

SLAM::Geometry::Point DestinationMessage::getPoint() const
{
    return point;
}

void DestinationMessage::serializeTo(QDataStream &stream) const
{
    stream << robotId << point;
}

void DestinationMessage::deserializeFrom(QDataStream &stream)
{
    stream >> robotId >> point;
}
}
