#ifndef SENDBIDSMESSAGE_H
#define SENDBIDSMESSAGE_H

#include "exploration/evaluationrecords.h"
#include "data/message.h"
#include "coordination/coordinationconstants.h"

namespace Data{    

    class SendBidsMessage : public Message, public Serializable
    {    
    public:
        SendBidsMessage();
        SendBidsMessage(int auctionId, int bidderId, QHash<uint, double> bids, bool isSender);

        ~SendBidsMessage();

        int getAuctionId() const;
        const QHash<uint,double> * getBids() const;
        int getBidderId() const;
        void serializeTo(QDataStream &stream) const;
        void deserializeFrom(QDataStream &stream);

    private:
        int auctionId; //the code that identify the current auction
        int bidderId;
        QHash<uint, double> bids;
        bool isSender;
    };
}

#endif // SENDBIDSMESSAGE_H
