#include "robotvictimcouplingmessage.h"

namespace Data{
RobotVictimCouplingMessage::RobotVictimCouplingMessage()
{
}

RobotVictimCouplingMessage::RobotVictimCouplingMessage(uint aVictimId, int aTimeToReach)
    : victimId(aVictimId), timeToReach(aTimeToReach)
{
}

RobotVictimCouplingMessage::~RobotVictimCouplingMessage()
{
}

uint RobotVictimCouplingMessage::getVictimId() const
{
    return victimId;
}

int RobotVictimCouplingMessage::getTimeToReach() const
{
    return timeToReach;
}

void RobotVictimCouplingMessage::serializeTo(QDataStream &stream) const
{
    stream << victimId << timeToReach;
}

void RobotVictimCouplingMessage::deserializeFrom(QDataStream &stream)
{
    stream >> victimId >> timeToReach;
}

}
