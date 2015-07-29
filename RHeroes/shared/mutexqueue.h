#ifndef MUTEXQUEUE_H
#define MUTEXQUEUE_H

#include <QObject>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
namespace Shared{

    template<class T>
    class MutexQueue
    {
    public:
        explicit MutexQueue();
        virtual ~MutexQueue();

        void enqueue(const T &obj);
        T dequeue();

        int size();

    private:
        /*
            This template combination is used to delete "variable" if and only if S is a pointer;
            It's used in order to allow auto-deletion of pointers to classes while at the same time
            making the code compile for non-pointer queue contents
        */
        template<class S> void pointerClear(S variable);
        template<class S> void pointerClear(S* variable);

        QQueue<T> *queue;
        QMutex *mutex;
        QWaitCondition *waitCond;
    };

    template<class T>
    MutexQueue<T>::MutexQueue() :
        queue(new QQueue<T>()), mutex(new QMutex()), waitCond(new QWaitCondition())
    {
    }

    template<class T>
    MutexQueue<T>::~MutexQueue()
    {
        waitCond->wakeAll();
        delete waitCond;
        delete mutex;
        while(!queue->isEmpty()){
            pointerClear(queue->dequeue());
            //delete queue->dequeue();
        }
        delete queue;
    }

    template<class T>
    void MutexQueue<T>::enqueue(const T & obj)
    {
        mutex->lock();
        queue->enqueue(obj);
        waitCond->wakeAll();
        mutex->unlock();
    }

    template<class T>
    T MutexQueue<T>::dequeue(){
        mutex->lock();
        //ldbg<<"lock"<<endl;
        while(queue->size() == 0){
            //ldbg<<"wait"<<endl;
            waitCond->wait(mutex);
        }
        //ldbg<<"unlock"<<endl;
        T toRet = queue->dequeue();
        mutex->unlock();

        return toRet;
    }

    template<class T>
    int MutexQueue<T>::size(){
        int toRet = 0;
        mutex->lock();
        toRet = queue->size();
        mutex->unlock();
        return toRet;
    }


    template <class T> template <class S>
    inline void MutexQueue<T>::pointerClear(S variable) {
        Q_UNUSED(variable)
    }

    template <class T> template <typename S>
    inline void MutexQueue<T>::pointerClear(S* variable) {
        delete variable;
    }
}

#endif // MUTEXQUEUE_H
