#ifndef ACKMESSAGE_H
#define ACKMESSAGE_H

#include <QDataStream>
#include "data/message.h"
#include "serializable.h"

namespace Data{
class AckMessage : public Message, public Serializable
{
public:
    AckMessage();
    AckMessage(int ackId);
    ~AckMessage();
    int getAckId() const;

    void serializeTo(QDataStream &stream) const;
    void deserializeFrom(QDataStream &stream);

private:
    int ackId;
};
}
#endif // ACKMESSAGE_H
