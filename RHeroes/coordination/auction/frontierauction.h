#ifndef FRONTIERAUCTION_H
#define FRONTIERAUCTION_H

#include "coordination/auction/auction.h"
#include "slam/geometry/frontier.h"
#include <QObject>

namespace Coordination{
    class FrontierAuction : public QObject, public Auction<SLAM::Geometry::Frontier>
    {
        Q_OBJECT
    public:
        FrontierAuction(QObject *parent = 0);
        ~FrontierAuction();        
        QHash<int, SLAM::Geometry::Frontier>  assignItems(); //the int variable in the QHash is the robotId

    signals:
        void auctionExpired();

    private:
        void onAuctionExpired();
    };
}

#endif // FRONTIERAUCTION_H
