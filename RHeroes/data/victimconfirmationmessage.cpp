#include "victimconfirmationmessage.h"

namespace Data{

VictimConfirmationMessage::VictimConfirmationMessage():
    position(Pose()), victimID(0), isNew(false)
{
}

VictimConfirmationMessage::VictimConfirmationMessage(Pose p, uint id, bool isNewVictim):
    position(p), victimID(id), isNew(isNewVictim)
{
}


VictimConfirmationMessage::~VictimConfirmationMessage()
{
}

const Pose &VictimConfirmationMessage::getPosition() const
{
    return position;
}

uint VictimConfirmationMessage::getVictimID() const
{
    return victimID;
}

bool VictimConfirmationMessage::isNewVictim() const
{
    return this->isNew;
}

void VictimConfirmationMessage::serializeTo(QDataStream &stream) const
{
    stream << position << victimID << isNew;
}

void VictimConfirmationMessage::deserializeFrom(QDataStream &stream)
{
    stream >> position >> victimID >> isNew;
}

}
