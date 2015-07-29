#ifndef VICTIMDELECTIONMESSAGE_H
#define VICTIMDELECTIONMESSAGE_H

#include "serializable.h"
#include "message.h"


namespace Data{
/**
 * @brief Victim detection message to Base Station
 *
 * VictimMessage is used as a communication container from robots to the base station.
 * It contains information about the robot generating the message (and that is supposed
 * to have find the victim), about the estimated position of the victim and about the
 * confidence of the detection process.
 *
 * @see
 */
class VictimDelectionMessage : public Message, public Serializable
{
public:
    VictimDelectionMessage();

    VictimDelectionMessage(uint id);

    const uint &getVictimID() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    uint victimID;
};

}
#endif // VICTIMDELECTIONMESSAGE_H
