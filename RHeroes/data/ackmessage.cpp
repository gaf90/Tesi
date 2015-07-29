#include "ackmessage.h"

namespace Data{
AckMessage::AckMessage()
{
}

AckMessage::AckMessage(int aAckId)
{
    ackId = aAckId;
}

AckMessage::~AckMessage()
{
}

int AckMessage::getAckId() const
{
    return ackId;
}

void AckMessage::serializeTo(QDataStream &stream) const{
    stream << ackId;
}

void AckMessage::deserializeFrom(QDataStream &stream){
    stream >> ackId;
}
}
