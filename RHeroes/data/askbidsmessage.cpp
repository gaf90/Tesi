#include "askbidsmessage.h"

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
    stream << auctionId << items << senderId;
    //TODO sistemare il passaggio dell'EvaluationRecords (da puntatore a "oggetto")
}

template<class T>
void AskBidsMessage<T>::deserializeFrom(QDataStream &stream){
    stream >> auctionId >> items >> senderId;
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
