#include "sendbidsmessage.h"


Data::SendBidsMessage::SendBidsMessage()
{
}


Data::SendBidsMessage::SendBidsMessage(int aAuctionId, int aBidderId, QHash<uint, double> aBids, bool aIsSender):
    auctionId(aAuctionId), bidderId(aBidderId), bids(aBids), isSender(aIsSender)
{
}


Data::SendBidsMessage::~SendBidsMessage()
{
    if (isSender){

    }else{

    }
    //TODO fare qlc
}


int Data::SendBidsMessage::getAuctionId() const
{
    return auctionId;
}


void Data::SendBidsMessage::serializeTo(QDataStream &stream) const
{
    stream << auctionId << bidderId << bids;
}


void Data::SendBidsMessage::deserializeFrom(QDataStream &stream)
{
    stream >> auctionId >> bidderId >> bids;
}


const QHash<uint, double> * Data::SendBidsMessage::getBids() const
{
    return &bids;
}


int Data::SendBidsMessage::getBidderId() const
{
    return bidderId;
}
