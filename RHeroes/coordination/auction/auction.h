#ifndef AUCTION_H
#define AUCTION_H

#include "data/message.h"
#include "data/objecttable.h"
#include "data/askbidsmessage.h"
#include "data/assignitemmessage.h"
#include "data/sendbidsmessage.h"
#include "data/ackmessage.h"
#include "data/wirelessmessage.h"
#include "coordination/coordinationconstants.h"
#include "data/buddymessage.h"
#include <QList>
#include <QHash>
#include <QObject>
#include <QTimer>

namespace Coordination{
    enum AuctionType {FRONTIER, VICTIM};

    /**
     * Class
     * bla bla bla
     */
    template<class T>
    class Auction
    {        
    public:
        /**
         * Constructor
         * bla bla bla
         */
        Auction(int timerInterval);
        virtual ~Auction();
        void startNewAuction();
        void endAuction();
        virtual QHash<int, T>  assignItems(); //remember to set auctionInProgress to false
        bool addItem(T * item);
        bool addItems(QList<T> * aItems);
        bool addBids(int robotId, QHash<uint,double> bids);
        int getAuctionId();
        QList<T> * getItems();
        QList<uint> & getItemsQHash();
        double getEvaluation(int robotId, uint item);

        bool isAuctionInProgress();
        void setAuctionType(AuctionType auctionType);
        AuctionType getAuctionType();
        Data::WirelessMessage createAskBidsMessage(int senderId);
        const Data::AskBidsMessage<T> * parseAskBidsMessage(const Data::BuddyMessage &msg);
        virtual Data::WirelessMessage createSendBidsMessage(int auctionId, int senderId, QHash<uint,double> bids, int auctioneerId);
        const Data::SendBidsMessage * parseSendBidsMessage(const Data::BuddyMessage &msg);
        Data::WirelessMessage createAssignItemMessage(T &item, int auctioneerRobotId, int destinationRobotId, int ackCode = 0);
        const Data::AssignItemMessage<T> * parseAssignItemMessage(const Data::BuddyMessage &msg);
        Data::WirelessMessage createAckMessage(int senderId, int destinationRobotId, int ackCode);    

    protected:
        QHash<uint, T> items; //uint:hashCode, T: items
        /*
                    item1 | item2 | ...
          robot1      X       Y
          robot2      Z       W
          ...
        */
        Data::ObjectTable<int, uint, double> bids; //int: robotId, uint item: hashcode, double: evaluation
        int auctionId;
        bool auctionInProgress;
        AuctionType auctionType;
        QTimer * auctionTimer;
        int timerInterval;
    };

    template<class T>
    Auction<T>::Auction(int aTimerInterval)
        : timerInterval(aTimerInterval), auctionTimer(new QTimer())
    {
        auctionTimer->setInterval(timerInterval);
        auctionTimer->setSingleShot(true);
        auctionInProgress = false;
        bids = Data::ObjectTable<int, uint, double>();
    }

    template<class T>
    Auction<T>::~Auction()
    {
    }

    template<class T>
    void Auction<T>::startNewAuction()
    {
        auctionTimer->start();
        auctionId = auctionTimer->timerId();
        auctionInProgress = true;
    }

    template<class T>
    void Auction<T>::endAuction()
    {
        auctionTimer->stop();
        items.clear();
        bids.clear();
        auctionInProgress = false; //theoretically already setted after assignItems()
        auctionId = -1;
    }

    template<class T>
    void Auction<T>::setAuctionType(AuctionType aAuctionType)
    {
        auctionType = aAuctionType;
    }

    template<class T>
    bool Auction<T>::addItems(QList<T> * aItems)
    {
        if (auctionInProgress){
            T item;
            foreach(item, *aItems)
                addItem(&item);
            return true;
        }
        return false;
    }

    template<class T>
    bool Auction<T>::addItem(T * item)
    {        
        if(auctionInProgress && !items.contains(qHash(item))){
            items.insert(qHash(*item), *item);
            return true;
        }
        return false;
    }

    template<class T>
    bool Auction<T>::addBids(int robotId, QHash<uint,double> aBids)
    {
        if (auctionInProgress && !bids.containsRow(robotId)){
            //there is an active auction and the robot (identified by robotId has not already bidded
            uint tempItem;            
            foreach(tempItem, items.keys()){
                if(!aBids.contains(tempItem))
                    return false;
            }
            bids.insertRow(robotId, aBids);
            return true;
        }
        return false;
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
        uint item;
        QList<T> * lstItem = new QList<T>;
        foreach(item, items.keys())
            lstItem->append(items[item]);
        return lstItem;
    }

    template<class T>
    QList<uint> & Auction<T>::getItemsQHash()
    {
        QList<uint> itemsQHash = items.keys();
        return itemsQHash;
    }

    template<class T>
    double Auction<T>::getEvaluation(int robotId, uint item)
    {
        return bids.getValue(robotId, item);
    }

    template<class T>
    AuctionType Auction<T>::getAuctionType()
    {
        return auctionType;
    }

    template<class T>
    QHash<int, T>  Auction<T>::assignItems()
    {
        QHash<int, T> hashReturn;
        return hashReturn;
    }

    using namespace Data;
    template<class T>
    WirelessMessage Auction<T>::createAskBidsMessage(int senderId)
    {
        Data::AskBidsMessage<T> *askBidsMessage = new Data::AskBidsMessage<T>(auctionId, getItems(), true, senderId);
        Data::BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(senderId), QString(""), askBidsMessage);
        Data::WirelessMessage message(buddy, Data::WirelessMessage::MessageBroadcastRobot);
        return message;
    }

    template<class T>
    const AskBidsMessage<T> * Auction<T>::parseAskBidsMessage(const Data::BuddyMessage &msg)
    {        
        return msg.get<AskBidsMessage<T> >();
    }   

    template<class T>
    WirelessMessage Auction<T>::createSendBidsMessage(int auctionId, int senderId, QHash<uint,double> bids, int auctioneerId)
    {        
        Data::BuddyMessage::Content content;
        if(auctionType == FRONTIER)
            content = Data::BuddyMessage::SendBidsOnFrontiers;
        else if (auctionType == VICTIM)
            content = Data::BuddyMessage::SendBidsOnVictims;
        SendBidsMessage *sendBidsMessage = new SendBidsMessage(auctionId, senderId, bids, true);
        BuddyMessage *buddy2 = new BuddyMessage(robotNameFromIndex(senderId), robotNameFromIndex(auctioneerId), sendBidsMessage, content);
        Data::WirelessMessage message(buddy2, WirelessMessage::MessageExchange);
        return message;
    }

    template<class T>
    const SendBidsMessage * Auction<T>::parseSendBidsMessage(const Data::BuddyMessage &msg)
    {        
        return msg.get<SendBidsMessage>();
    }

    template<class T>
    WirelessMessage Auction<T>::createAssignItemMessage(T &item, int auctioneerRobotId, int destinationRobotId, int ackCode)
    {
        AssignItemMessage<T> *assignItemMessage = new AssignItemMessage<T>(item, true, ackCode);
        BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(auctioneerRobotId), robotNameFromIndex(destinationRobotId), assignItemMessage);
        Data::WirelessMessage message(buddy, WirelessMessage::MessageExchange);
        return message;
    }


    template<class T>
    const AssignItemMessage<T> * Auction<T>::parseAssignItemMessage(const Data::BuddyMessage &msg)
    {        
        return msg.get<AssignItemMessage<T> >();
    }

    template<class T>
    Data::WirelessMessage Auction<T>::createAckMessage(int senderId, int destinationRobotId, int ackCode)
    {
        AckMessage *ackMessage = new AckMessage(ackCode);
        BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(senderId), robotNameFromIndex(destinationRobotId), ackMessage);
        Data::WirelessMessage message(buddy, WirelessMessage::MessageExchange);
        return message;
    }


}
#endif // AUCTION_H
