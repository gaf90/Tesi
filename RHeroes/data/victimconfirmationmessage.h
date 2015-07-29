#ifndef VICTIMCONFIRMATIONMESSAGE_H
#define VICTIMCONFIRMATIONMESSAGE_H

#include "serializable.h"
#include "message.h"
#include "data/pose.h"

namespace Data{
/**
 * @brief Victim detection message from Base station to single robot
 *
 * VictimConfirmationMessage is used to confirm the presence of a victim to a specific robot.
 * The message is sent to the first discoverer of the victim and the base station waits for an
 * ackMessage to confirm. If the ack has not been received, the Base Station sends the message again,
 * after a timeout = ACK_TIMEOUT, for ACK_TENTATIVES times. If no ack has been received, the message
 * will be sent to another robot with the same procedure. This will be handled by the
 * messageDeliverer module.
 *
 * @see
 */
class VictimConfirmationMessage : public Message, public Serializable
{
public:
    VictimConfirmationMessage();

    VictimConfirmationMessage(Pose p, uint id, bool isNewVictim);

    /**
     * Destroys the VictimConfirmationMessage
     */
    virtual ~VictimConfirmationMessage();

    const Pose &getPosition() const;
    uint getVictimID() const;
    bool isNewVictim() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    Pose position;
    uint victimID;
    bool isNew;
};

}
#endif // VICTIMCONFIRMATIONMESSAGE_H
