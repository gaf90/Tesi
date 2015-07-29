#ifndef WIRELESSMESSAGE_H
#define WIRELESSMESSAGE_H

#include "message.h"
#include "buddymessage.h"
#include <QUuid>

namespace Data {

class WirelessMessage : public Message
{
public:
    enum Command {
        NeighbourQuery, //TODO: da fare
        NeighbourAnswer, //TODO: da fare
        SignalQuery,
        SignalAnswer,
        MessageExchange,
        MessageBroadcast,
        MessageBroadcastRobot
    };

    WirelessMessage();
    WirelessMessage(WirelessMessage::Command command);
    WirelessMessage(const BuddyMessage *msg,
                    WirelessMessage::Command messageType = MessageExchange);
    virtual ~WirelessMessage();

    WirelessMessage::Command getCommand() const;
    void setWirelessCommand(WirelessMessage::Command command);

    const BuddyMessage *getBuddyMessage() const;
    void setBuddyMessage(const BuddyMessage *msg);

    void setRobotIdForSignalStrength(uint id);
    uint getRobotIdForSignalStrength() const;

    void  setSignalPower(double signalPower);
    double getSignalPower() const;

private:
    Command command;
    const BuddyMessage *msg;
    uint robotIdForSignalStrength;
    double signalPower;
};

}

#endif // WIRELESSMESSAGE_H
