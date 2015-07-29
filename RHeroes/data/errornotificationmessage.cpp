#include "errornotificationmessage.h"

namespace Data{

ErrorNotificationMessage::ErrorNotificationMessage():
    module(ErrorNotificationMessage::Unknown)
{
}

ErrorNotificationMessage::ErrorNotificationMessage(InvolvedModule mod):
    module(mod)
{
}

ErrorNotificationMessage::~ErrorNotificationMessage()
{
}

void ErrorNotificationMessage::serializeTo(QDataStream &stream) const
{
    quint8 module_id = this->module;
    stream << module_id;
}

void ErrorNotificationMessage::deserializeFrom(QDataStream &stream)
{
    quint8 module_id;
    stream >> module_id;
    this->module = (ErrorNotificationMessage::InvolvedModule) module_id;
}

ErrorNotificationMessage::InvolvedModule ErrorNotificationMessage::getInvolvedModule() const
{
    return this->module;
}

}
