#ifndef ERRORNOTIFICATIONMESSAGE_H
#define ERRORNOTIFICATIONMESSAGE_H

#include "data/message.h"
#include "data/serializable.h"

namespace Data{

class ErrorNotificationMessage : public Message, public Serializable
{
public:
    enum InvolvedModule{
        PathPlanner,
        Exploration,
        Navigation,
        VictimDetection,
        MapKO,
        Unknown
    };

    ErrorNotificationMessage();

    ErrorNotificationMessage(ErrorNotificationMessage::InvolvedModule mod);

    ErrorNotificationMessage::InvolvedModule getInvolvedModule() const;

    virtual ~ErrorNotificationMessage();

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    InvolvedModule module;
};

}
#endif // ERRORNOTIFICATIONMESSAGE_H
