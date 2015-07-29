#include "distancevectormessage.h"
#include "shared/constants.h"

namespace Data{

DistanceVectorMessage::DistanceVectorMessage():
    sender("none"), connectedPeers()
{
}

DistanceVectorMessage::DistanceVectorMessage(QString sender, QHash<QString, int> peers):
    sender(sender), connectedPeers(peers)
{
//    connectedPeers = TYPE_DISTANCE_VECTOR;
//    foreach(QString peer, peers.keys())
//    {
//        connectedPeers += " ";
//        connectedPeers += "{";
//        connectedPeers += peer;
//        connectedPeers += " ";
//        connectedPeers += QString::number(peers[peer]);
//        connectedPeers += "}";
//    }
}

DistanceVectorMessage::~DistanceVectorMessage()
{
}

const QHash<QString, int> &DistanceVectorMessage::getReachablePeers() const
{
    return connectedPeers;
}

const QString &DistanceVectorMessage::getSender() const
{
    return sender;
}

void DistanceVectorMessage::serializeTo(QDataStream &stream) const
{
    stream << sender << connectedPeers;
}

void DistanceVectorMessage::deserializeFrom(QDataStream &stream)
{
    stream >> sender >> connectedPeers;
}

}
