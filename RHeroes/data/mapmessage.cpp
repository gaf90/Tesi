#include "mapmessage.h"


namespace Data{
MapMessage::MapMessage():
    map(),
    estimated(true)
{
}

MapMessage::MapMessage(SLAM::Map mappa, bool estimated):
    map(mappa),
    estimated(estimated)
{
}

MapMessage::~MapMessage()
{
}
void MapMessage::serializeTo(QDataStream &stream) const
{
    stream << map << estimated;
}

void MapMessage::deserializeFrom(QDataStream &stream)
{
    stream >> map >> estimated;
}



}

SLAM::Map Data::MapMessage::getMap() const
{
    return this->map;
}

bool Data::MapMessage::isEstimated() const
{
    return estimated;
}
