#ifndef DISTANCEVECTORMESSAGE_H
#define DISTANCEVECTORMESSAGE_H

#include "serializable.h"
#include "message.h"
#include <QHash>

namespace Data{
/**
* Distance vector message used to establish routing paths between peers
*/
class DistanceVectorMessage: public Serializable, Message
{
public:
    DistanceVectorMessage();

    /**
    * Builds a Distance vector.
    *
    * @param sender, the name of the peer that sent the distance vector.
    * @param peers, the Qhash containint the reachable peers from the sender and relative cost.
    */
    DistanceVectorMessage(QString sender, QHash<QString, int> peers);

    /**
    * Destroys the DistanceVector message
    */
    virtual ~DistanceVectorMessage();

    /**
    * Returns the QHash structure containing the set of reachable peers and their cost
    */
    const QHash<QString, int> &getReachablePeers() const;

    /**
    * Returns the Distance vector sender's name
    */
    const QString &getSender() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    //! The sender of the message
    QString sender;

    //! Distance vector information, with the table of reachable peers and relative cost
    QHash<QString, int> connectedPeers;
};

}
#endif // DISTANCEVECTORMESSAGE_H
