#ifndef ASKBIDSMESSAGE_H
#define ASKBIDSMESSAGE_H

#include "data/message.h"
#include "serializable.h"
#include "exploration/evaluationrecords.h"
#include "slam/geometry/frontier.h"
#include "coordination/coordinationconstants.h"

namespace Data {

    template<class T>
    class AskBidsMessage : public Message, public Serializable
    {
    public:

        AskBidsMessage();
        AskBidsMessage(int aAuctionId, QList<T> * items, bool isSender, int senderId);

        ~AskBidsMessage();

        int getAuctionId() const;
        QList<T> * getItems() const;
        int getSenderId() const;        

        void serializeTo(QDataStream &stream) const;
        void deserializeFrom(QDataStream &stream);

    private:
        QList<T> * items;
        int auctionId; //the code that identify the current auction
        bool isSender;
        int senderId;
    };

    using namespace Data;

    template<class T>
    AskBidsMessage<T>::AskBidsMessage() :
        items(new QList<T>), auctionId(0)
    {
    }

    template<class T>
    AskBidsMessage<T>::AskBidsMessage(int aAuctionId, QList<T> * aItems,
                                   bool aIsSender, int aSenderId):
        isSender(aIsSender),auctionId(aAuctionId), items(aItems), senderId(aSenderId)
    {
    }

    template<class T>
    AskBidsMessage<T>::~AskBidsMessage(){
        if (isSender){

        }else{

        }
        //TODO fare qlc
    }

    template<class T>
    int AskBidsMessage<T>::getAuctionId() const
    {
        return auctionId;
    }

    template<class T>
    void AskBidsMessage<T>::serializeTo(QDataStream &stream) const{
        stream << auctionId << *items << senderId;        
    }

    template<class T>
    void AskBidsMessage<T>::deserializeFrom(QDataStream &stream){
        stream >> auctionId >> *items >> senderId;
    }

    template<class T>
    QList<T> * AskBidsMessage<T>::getItems() const
    {
        return items;
    }

    template<class T>
    int AskBidsMessage<T>::getSenderId() const
    {
        return senderId;
    }
}

#endif // ASKBIDSMESSAGE_H
