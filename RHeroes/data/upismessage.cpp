#include "upismessage.h"

namespace Data {

UPISMessage::UPISMessage(bool jpeg, const QByteArray &binData) :
    QByteArray(binData), jpeg(jpeg)
{
}

UPISMessage::~UPISMessage()
{
}

bool UPISMessage::isJPEG() const
{
    return jpeg;
}

bool UPISMessage::isRAW() const
{
    return !jpeg;
}

}
