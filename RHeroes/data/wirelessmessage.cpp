#include "wirelessmessage.h"

namespace Data {

WirelessMessage::WirelessMessage()
{
}

WirelessMessage::WirelessMessage(Command command) :
    command(command), msg(NULL)
{
}

WirelessMessage::WirelessMessage(const BuddyMessage *msg, Command messageType) :
    command(messageType), msg(msg)
{
}

WirelessMessage::~WirelessMessage()
{
}

WirelessMessage::Command WirelessMessage::getCommand() const
{
    return command;
}

void WirelessMessage::setWirelessCommand(
    WirelessMessage::Command command)
{
    this->command = command;
}

const BuddyMessage *WirelessMessage::getBuddyMessage() const
{
    return msg;
}

void WirelessMessage::setBuddyMessage(const BuddyMessage *msg)
{
    this->msg = msg;
}

void WirelessMessage::setRobotIdForSignalStrength(uint id)
{
    robotIdForSignalStrength = id;
}

uint WirelessMessage::getRobotIdForSignalStrength() const
{
    return robotIdForSignalStrength;
}

void WirelessMessage::setSignalPower(double signalPower)
{
    this->signalPower = signalPower;
}

double WirelessMessage::getSignalPower() const
{
    return signalPower;
}

}
