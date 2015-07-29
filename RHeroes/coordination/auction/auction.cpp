#include "coordination/auction/auction.h"
#include "data/askbidsmessage.h"
#include "data/buddymessage.h"
#include "shared/utilities.h"

namespace Coordination{
    template<class T>
    Auction<T>::Auction()
    {
        auctionInProgress = false;
    }

    template<class T>
    void Auction<T>::startNewAuction(int aAuctionId)
    {
        auctionId = aAuctionId;
        auctionInProgress = true;
    }

    template<class T>
    void Auction<T>::endAuction()
    {
        items.clear();
        bids.clear();
        auctionInProgress = false; //theoretically already setted after assign items
        auctionId = -1;
    }

    template<class T>
    void Auction<T>::setAuctionType(AuctionType aAuctionType)
    {
        auctionType = aAuctionType;
    }

    template<class T>
    void Auction<T>::addItems(QList<T> * aItems)
    {
        T item;
        foreach(item, aItems)
            addItem(item);
    }

    template<class T>
    void Auction<T>::addItem(T item)
    {
        items.append(item.hashCode(), item);
    }

    template<class T>
    void Auction<T>::addBids(int robotId, QHash<uint,double> aBids)
    {
        bids.insertRow(robotId, aBids);
    }

    template<class T>
    int Auction<T>::getAuctionId()
    {
        return auctionId;
    }

    template<class T>
    bool Auction<T>::isAuctionInProgress()
    {
        return auctionInProgress;
    }

    template<class T>
    QList<T> * Auction<T>::getItems()
    {
        return items;
    }

    template<class T>
    AuctionType Auction<T>::getAuctionType()
    {
        return auctionType;
    }

    using namespace Data;
    template<class T>
    WirelessMessage Auction<T>::createAskBidsMessage(int senderId)
    {
        Data::AskBidsMessage<T> *askBidsMessage = new Data::AskBidsMessage<T>(auctionId, *items, true, senderId);
        Data::BuddyMessage *buddy = new Data::BuddyMessage(robotNameFromIndex(senderId), QString(""), askBidsMessage);
        Data::WirelessMessage message(buddy, Data::WirelessMessage::MessageBroadcastRobot);
        return message;
    }

    /*template<class T>
    const AskBidsMessage<T> Auction::parseAskBidsMessage(Data::Message msg)
    {
        const WirelessMessage &mess = (const WirelessMessage &)msg;
        const BuddyMessage *buddy = mess.getBuddyMessage();
        return buddy->getAskBidsMessage();
    }

    template<class T>
    WirelessMessage Auction<T>::createSendBidsMessage(int auctionId, int senderId, QHash<Coordination::T, double> bids, int auctioneerId)
    {
        SendBidsMessage *sendBidsMessage = new SendBidsMessage(auctionId, senderId, evaluationRecords->getEvaluations(), true);
        BuddyMessage *buddy2 = new BuddyMessage(robotNameFromIndex(senderId), robotNameFromIndex(auctioneerId), sendBidsMessage);
        return message(buddy2, WirelessMessage::MessageExchange);
    }

    template<class T>
    const SendBidsMessage<Coordination::T> Auction::parseSendBidsMessage(Data::WirelessMessage msg)
    {
        const WirelessMessage &mess = (const WirelessMessage &)msg;
        const BuddyMessage *buddy = mess.getBuddyMessage();
        return buddy->getSendBidsMessage();
    }

    WirelessMessage Auction::createAssignItemMessage(T item, int auctioneerRobotId,int destionationRobotId)
    {
        AssignItemMessage<SLAM::Geometry::Frontier *> *assignFrontierMessage = new AssignItemMessage<SLAM::Geometry::Frontier *>(item, true);
        BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(auctioneerRobotId), robotNameFromIndex(destinationRobotId), assignFrontierMessage);
        return message(buddy, WirelessMessage::MessageExchange) ;
    }

    const AssignItemMessage<T> Auction::parseAssignItemMessage(Data::WirelessMessage msg)
    {
        const WirelessMessage &mess = (const WirelessMessage &)msg;
        const BuddyMessage *buddy = mess.getBuddyMessage();
        return buddy->getAssignItemOnFrontierMessage();
    }
    */
}

