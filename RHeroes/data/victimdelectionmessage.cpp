#include "victimdelectionmessage.h"
#include "baseStation/params.h"

namespace Data{

VictimDelectionMessage::VictimDelectionMessage():
    victimID(NO_VICTIM)
{
}

VictimDelectionMessage::VictimDelectionMessage(uint id):
    victimID(id)
{
}

const uint &VictimDelectionMessage::getVictimID() const
{
    return victimID;
}

void VictimDelectionMessage::serializeTo(QDataStream &stream) const
{
    stream << victimID;
}

void VictimDelectionMessage::deserializeFrom(QDataStream &stream)
{
    stream >> victimID;
}

}
