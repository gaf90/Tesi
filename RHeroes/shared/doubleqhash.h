#ifndef DOUBLEQHASH_H
#define DOUBLEQHASH_H

#include <QHash>

namespace Shared{
    template <class T1, class T2, class T3>
    class DoubleQHash
    {
    public:
        DoubleQHash();
        ~DoubleQHash();

        void insert(T1 firstKey, T2 secondKey, T3 elem);
        const T3 & getByFirstKey(T1 key);
        const T3 & getBySecondKey(T2 key);
        bool containsByFirstKey(T1 key);
        bool containsBySecondKey(T2 key);
        void removeByFirstKey(T1 firstKey);
        void removeBySecondKey(T2 secondKey);
        /*void modifyFirstKey(T1 firstKey, T3 elem);
        void modifySecondKey(T2 secondKey, T3 elem);*/


    private:
        /*T1 & getFirstKey(T3 elem);
        T2 & getSecondKey(T3 elem);*/

        QHash<T1,T3> firstKeyQHash;
        QHash<T2,T3> secondKeyQHash;
        QHash<T1,T2> firstToSecondKey;
        QHash<T2,T1> secondToFirstKey;
    };

    template<class T1, class T2, class T3>
    DoubleQHash<T1,T2,T3>::DoubleQHash()
    {
    }

    template<class T1, class T2, class T3>
    DoubleQHash<T1,T2,T3>::~DoubleQHash()
    {
    }

    template<class T1, class T2, class T3>
    void DoubleQHash<T1,T2,T3>::insert(T1 firstKey, T2 secondKey, T3 elem)
    {
        firstKeyQHash.insert(firstKey, elem);
        secondKeyQHash.insert(secondKey, elem);
        firstToSecondKey.insert(firstKey, secondKey);
        secondToFirstKey.insert(secondKey, firstKey);
    }

    template<class T1, class T2, class T3>
    const T3 & DoubleQHash<T1,T2,T3>::getByFirstKey(T1 key)
    {
        return firstKeyQHash[key];
    }

    template<class T1, class T2, class T3>
    const T3 & DoubleQHash<T1,T2,T3>::getBySecondKey(T2 key)
    {
        return secondKeyQHash[key];
    }

    template<class T1, class T2, class T3>
    bool DoubleQHash<T1,T2,T3>::containsByFirstKey(T1 key)
    {
        return firstToSecondKey.contains(key);
    }

    template<class T1, class T2, class T3>
    bool DoubleQHash<T1,T2,T3>::containsBySecondKey(T2 key)
    {
        return secondToFirstKey.contains(key);
    }

    template<class T1, class T2, class T3>
    void DoubleQHash<T1,T2,T3>::removeByFirstKey(T1 firstKey)
    {
        T2 secondKey = firstToSecondKey[firstKey];
        firstKeyQHash.remove(firstKey);
        secondKeyQHash.remove(secondKey);
        firstToSecondKey.remove(firstKey);
        secondToFirstKey.remove(secondKey);
    }

    template<class T1, class T2, class T3>
    void DoubleQHash<T1,T2,T3>::removeBySecondKey(T2 secondKey)
    {
        T1 firstKey = secondToFirstKey[secondKey];
        firstKeyQHash.remove(firstKey);
        secondKeyQHash.remove(secondKey);
        firstToSecondKey.remove(firstKey);
        secondToFirstKey.remove(secondKey);
    }

    /*template<class T1, class T2, class T3>
    DoubleQHash<T1,T2,T3>::modifyFirstKey(T1 firstKey, T3* elem)
    {
        firstKeyQHash.remove(getFirstKey(elem));
        firstKeyQHash.insert(firstKey, elem);
    }

    DoubleQHash<T1,T2,T3>::modifySecondKey(T2 secondKey, T3* elem)
    {
        secondKeyQHash.remove(getSecondtKey(elem));        
        secondKeyQHash.insert(secondKey, elem);
    }*/

    /*template<class T1, class T2, class T3>
    T1 & DoubleQHash<T1,T2,T3>::getFirstKey(T3 elem)
    {
        T1 tempElem;
        foreach(tempElem, firstKeyQHash.keys())
        {
            if(firstKeyQHash[tempElem] == tempElem)
                return tempElem;
        }
    }

    template<class T1, class T2, class T3>
    T2 & DoubleQHash<T1,T2,T3>::getSecondKey(T3 elem)
    {
        T2 tempElem;
        foreach(tempElem, secondKeyQHash.keys())
        {
            if(secondKeyQHash[tempElem] == tempElem)
                return tempElem;
        }
    }*/
}
#endif // DOUBLEQHASH_H
