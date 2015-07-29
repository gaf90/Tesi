#ifndef ASSIGNITEMMESSAGE_H
#define ASSIGNITEMMESSAGE_H

#include "data/message.h"
#include "serializable.h"

namespace Data{

    template <class T>
    class AssignItemMessage : public Message, public Serializable
    {
    public:
        AssignItemMessage();
        AssignItemMessage(T item, bool isSender, int aAckCode = 0, int aEvaluation = 0);

        ~AssignItemMessage();

        const T getItem() const;
        int getAckCode() const;
        int getEvaluation() const;

        void serializeTo(QDataStream &stream) const;
        void deserializeFrom(QDataStream &stream);

    private:
        T item;
        int ackCode;
        bool isSender;
        int evaluation;
    };

    template <class T>
    AssignItemMessage<T>::AssignItemMessage()
    {
    }

    template <class T>
    AssignItemMessage<T>::AssignItemMessage(T aItem, bool aIsSender, int aAckCode, int aEvaluation) :
        isSender(aIsSender), item(aItem), ackCode(aAckCode), evaluation(aEvaluation)
    {
    }

    template <class T>
    AssignItemMessage<T>::~AssignItemMessage()
    {
        if (isSender){

        }else{

        }
        //TODO fare qlc
    }

    template <class T>
    const T AssignItemMessage<T>::getItem() const
    {
        return item;
    }

    template <class T>
    int AssignItemMessage<T>::getAckCode() const
    {
        return ackCode;
    }

    template <class T>
    int AssignItemMessage<T>::getEvaluation() const
    {
        return evaluation;
    }

    template <class T>
    void AssignItemMessage<T>::serializeTo(QDataStream &stream) const
    {
        stream << item << ackCode << evaluation;
    }

    template <class T>
    void AssignItemMessage<T>::deserializeFrom(QDataStream &stream)
    {
        stream >> item >> ackCode << evaluation;
    }

}
#endif // ASSIGNITEMMESSAGE_H
