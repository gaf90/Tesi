//#include "assignitemmessage.h"

//namespace Data{
//    template <class T>
//    AssignItemMessage<T>::AssignItemMessage()
//    {
//    }

//    template <class T>
//    AssignItemMessage<T>::AssignItemMessage(T *aItem, bool aIsSender) :
//        isSender(aIsSender), item(aItem)
//    {
//    }

//    template <class T>
//    AssignItemMessage<T>::~AssignItemMessage()
//    {
//        if (isSender){

//        }else{

//        }
//        //TODO fare qlc
//    }

//    template <class T>
//    T * AssignItemMessage<T>::getItem() const
//    {
//        return item;
//    }

//    template <class T>
//    void AssignItemMessage<T>::serializeTo(QDataStream &stream) const
//    {
//        stream << item;
//    }

//    template <class T>
//    void AssignItemMessage<T>::deserializeFrom(QDataStream &stream)
//    {
//        stream >> item;
//    }
//}
