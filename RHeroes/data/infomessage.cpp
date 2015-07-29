#include "infomessage.h"

namespace Data{
InfoMessage::InfoMessage():
    infoName("none"),
    info("none")
{
}

InfoMessage::~InfoMessage()
{
}

InfoMessage::InfoMessage(QString infoName):
        infoName(infoName),
        info("none")
{
}

InfoMessage::InfoMessage(QString infoName, QString info):
    infoName(infoName),
    info(info)
{
}


QString InfoMessage::getInfoName() const
{
    return infoName;
}

QString InfoMessage::getInfo() const
{
    return info;
}

void InfoMessage::serializeTo(QDataStream &stream) const
{
    stream << infoName << info;
}

void InfoMessage::deserializeFrom(QDataStream &stream)
{
    stream >> infoName >> info;
}


}



