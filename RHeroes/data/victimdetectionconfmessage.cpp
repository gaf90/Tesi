#include "victimdetectionconfmessage.h"


namespace Data{

VictimDetectionConfMessage::VictimDetectionConfMessage()
{
}

VictimDetectionConfMessage::VictimDetectionConfMessage(double hMin, double sMin,
                                                       double vMin, double hMax,
                                                       double sMax, double vMax):
    hMin(hMin), sMin(sMin), vMin(vMin), hMax(hMax), sMax(sMax), vMax(vMax)
{
}

VictimDetectionConfMessage::~VictimDetectionConfMessage()
{
}

void VictimDetectionConfMessage::serializeTo(QDataStream &stream) const
{
    stream << hMin << sMin << vMin << hMax << sMax << vMax;
}

void VictimDetectionConfMessage::deserializeFrom(QDataStream &stream)
{
    stream >> hMin >> sMin >> vMin >> hMax >> sMax >> vMax;
}

double VictimDetectionConfMessage::getHMin() const
{
    return hMin;
}

double VictimDetectionConfMessage::getSMin() const
{
    return sMin;
}

double VictimDetectionConfMessage::getVMin() const
{
    return vMin;
}

double VictimDetectionConfMessage::getHMax() const
{
    return hMax;
}

double VictimDetectionConfMessage::getSMax() const
{
    return sMax;
}

double VictimDetectionConfMessage::getVMax() const
{
    return vMax;
}

}
