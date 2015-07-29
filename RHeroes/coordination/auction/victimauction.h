#ifndef VICTIMAUCTION_H
#define VICTIMAUCTION_H

#include "coordination/auction/auction.h"
#include "data/pose.h"
#include <QObject>

namespace Coordination{
    class VictimAuction : public QObject, public Auction<uint>
    {
        Q_OBJECT
    public:
        VictimAuction(QObject *parent = 0);
        ~VictimAuction();
        QHash<int, uint>  assignItems(); //the int variable in the QHash is the robotId, the uint is the victimId

    signals:
        void auctionExpired();

    private:
        void onAuctionExpired();
    };
}
#endif // VICTIMAUCTION_H
