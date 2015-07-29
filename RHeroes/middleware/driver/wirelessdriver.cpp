#include "wirelessdriver.h"
#include "data/wssmessage.h"
#include "data/distancevectormessage.h"
#include "shared/utilities.h"
#include "shared/config.h"
#include <typeinfo>
#include <QDataStream>
#include <stdint.h>
#include <QDebug>

namespace Middleware {

using namespace Data;

#warning Gestire NeighbourQuery

WirelessDriver::WirelessDriver(QObject *parent, uint nrobot) :
    SensorDriver(parent), nRobot(nrobot)
{
}

WirelessDriver::~WirelessDriver()
{
}

void WirelessDriver::sendMessage(
        const WirelessMessage &msg, const QString &destination)
{
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    const BuddyMessage *buddy = msg.getBuddyMessage();
    stream << buddy;

    WSSMessage wssmsg;
    wssmsg.setRobotData(destination, &data);
    if(buddy->getContent() == BuddyMessage::VictimConfirmation ||
            buddy->getContent() == BuddyMessage::VictimDeletion ||
            buddy->getContent() == BuddyMessage::VictimAssignment ||
            buddy->getContent() == BuddyMessage::ModuleActivation)
        wssmsg.setDeliveringMode(true);
    else
        wssmsg.setDeliveringMode(false);
    emit sigDriveMessageSend(wssmsg);
}

void WirelessDriver::sendMessageBroadcastRobot(const Data::WirelessMessage &msg)
{
    ldbg << "i believe there are " << Config::robotCount << " robots" << endl;
    for(int i = 0; i < Config::robotCount; i++) {
        QString dest = robotNameFromIndex(i);
        if(msg.getBuddyMessage()->getSource() != dest) {
            sendMessage(msg, robotNameFromIndex(i));
        }
    }
}

void WirelessDriver::onDriverMessage(const Message &message)
{
    if(typeid(message) == typeid(const WirelessMessage &)) {
        const WirelessMessage &msg = (const WirelessMessage &) message;

        if(msg.getCommand() == WirelessMessage::MessageExchange) {
            sendMessage(msg, msg.getBuddyMessage()->getDestination());
        } else if(msg.getCommand() == WirelessMessage::MessageBroadcast) {
            sendMessageBroadcastRobot(msg);
            sendMessage(msg, robotNameFromIndex(BASE_STATION_ID));
        } else if(msg.getCommand() == WirelessMessage::MessageBroadcastRobot) {
            sendMessageBroadcastRobot(msg);
        } if(msg.getCommand() == WirelessMessage::SignalQuery){
            uint idToQuery = msg.getRobotIdForSignalStrength();
            WSSMessage wssmsg;
            wssmsg.setType("GETSS");
            wssmsg.insert("Robot", robotNameFromIndex(idToQuery));
            emit sigDriveMessageSend(wssmsg);
        }
    }
}

void WirelessDriver::onMessageReceived(const Message &message)
{
    if(typeid(message) == typeid(const WSSMessage &)) {
        const WSSMessage &msg = (const WSSMessage &) message;
        const QString &src = msg.getPeer();

        if(msg.getType() == WSSMessage::TYPE_ROBOT_DATA) {

            int dataSize = msg.getRobotData()->length();
            const QByteArray *data;

            if(partialData.contains(src)) {
                partialData[src].append(*msg.getRobotData());
                data = &partialData[src];
                dataSize = data->length();
            } else {
                data = msg.getRobotData();
            }

            QDataStream stream(*data);
            QIODevice *device = stream.device();
            stream.setByteOrder(QDataStream::BigEndian);

            while(!stream.atEnd()) {
                uint32_t objSize;
                stream >> objSize;
                if(objSize>2044){
                    qDebug("LOST PACKET - %i",objSize);
                }
                int dataLeft = dataSize - device->pos();
                if(dataLeft < (int) objSize) {
                    partialData[src] =
                            data->right(dataLeft + sizeof(uint32_t));
                    break;
                } else {
                    BuddyMessage buddy;
                    device->seek(device->pos() - sizeof(uint32_t));
                    stream >> buddy;
//                    if(stream.atEnd())
//                    {
                        if(buddy.getDestination() == "" || buddy.getDestination() == robotNameFromIndex(nRobot)) {
                            //Qui gestione DV
                            if(buddy.getContent() == BuddyMessage::DistanceVector) {
                                const DistanceVectorMessage * message = buddy.getDistanceVector();
                                emit sigDistanceVector(message->getSender(), message->getReachablePeers());
                            } else {
                                emit sigSensorData(WirelessMessage(&buddy));
                            }
                        } else {
                            onDriverMessage(WirelessMessage(&buddy));
                        }
//                    }
                }
            }
        } else if(msg.getType() == "SS"){
            WirelessMessage message(WirelessMessage::SignalAnswer);
            message.setRobotIdForSignalStrength(robotIndexFromName(msg["Robot"]));
            message.setSignalPower(msg["Strength"].toDouble());
            emit sigSensorData(message);
        }
        else {
            // Ignore uninteresting messages
        }
    }
}

void WirelessDriver::flushBuffer(const QString &robotName){
    if(partialData.contains(robotName)){
        partialData.remove(robotName);
    }
}

}
