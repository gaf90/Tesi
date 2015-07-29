#ifndef COORDINATIONMESSAGEMODULE_H
#define COORDINATIONMESSAGEMODULE_H

#include <QObject>
#include <QThread>
#include <QHash>
#include <QSignalMapper>

#include "shared/doubleqhash.h"
#include "data/wirelessmessage.h"
#include "shared/mutexqueue.h"

namespace Coordination {
    class CoordinationModule;
}


namespace Coordination{
    class CoordinationMessageModule : public QThread
    {
    Q_OBJECT
    public:
        explicit CoordinationMessageModule(CoordinationModule * coordinationModule, QObject *parent = 0);
        void setCoordinationMessageQueue(Shared::MutexQueue<const Data::BuddyMessage *> * aCoordinationMessageQueue);
        Shared::MutexQueue<const Data::BuddyMessage *> *getBidsQueue();

    signals:
        void sigMessageSendCMM(const Data::Message &msg);

    public slots:
        void onAddNewMessage(const Data::Message &msg, bool needAck = false, int resendInterval = 0, int ackCode = 0);
        virtual void start(); //starting the thread

    private slots:
        void onSingleMessageTimerExpired(int timerId);
        void onCheckTimerMessageExpired();

    protected:
        void run();

    private:
        void onAckMessage(const Data::AckMessage &ackMessage);
        Shared::DoubleQHash<int, int, Data::Message> messages;
        Shared::MutexQueue<const Data::BuddyMessage *> * coordinationMessageQueue;
        QSignalMapper *messageTimeoutMapper;
        QTimer *checkNewMessageInQueueTimer;
        CoordinationModule *coordinationModule;
        QHash<int, QTimer *> messageQTimerHash; //int is the timerId, necessary to delete timers after ack received
    };
}

#include "coordination/coordinationmodule.h"

#endif // COORDINATIONMESSAGEMODULE_H
