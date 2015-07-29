#ifndef DESTINATIONMESSAGE_H
#define DESTINATIONMESSAGE_H

#include <QDataStream>
#include "data/message.h"
#include "serializable.h"
#include "slam/geometry/point.h"

namespace Data{
class DestinationMessage : public Message, public Serializable
{
public:
    DestinationMessage();
    DestinationMessage(int robotId, SLAM::Geometry::Point point);
    ~DestinationMessage();

    int getRobotId() const;
    SLAM::Geometry::Point getPoint() const;

    void serializeTo(QDataStream &stream) const;
    void deserializeFrom(QDataStream &stream);

private:
    int robotId;
    SLAM::Geometry::Point point;
};
}

#endif // DESTINATIONMESSAGE_H
