#ifndef VICTIMDETECTIONCONFMESSAGE_H
#define VICTIMDETECTIONCONFMESSAGE_H

#include "data/message.h"
#include "data/serializable.h"

namespace Data{

class VictimDetectionConfMessage : public Message, public Serializable
{
public:
    VictimDetectionConfMessage();

    VictimDetectionConfMessage(double hMin, double sMin, double vMin, double hMax, double sMax, double vMax);

    /**
    * Destroys the VictimDetectionConfMessage object.
    */
    virtual ~VictimDetectionConfMessage();

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    double getHMin() const;
    double getSMin() const;
    double getVMin() const;
    double getHMax() const;
    double getSMax() const;
    double getVMax() const;

private:

    double hMin;
    double sMin;
    double vMin;
    double hMax;
    double sMax;
    double vMax;


};

}
#endif // VICTIMDETECTIONCONFMESSAGE_H
