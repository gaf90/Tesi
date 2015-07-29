#include "coordinationmessagemodule.h"
#include "coordination/coordinationconstants.h"
#include "data/ackmessage.h"

#include <QTimer>

namespace Coordination{
CoordinationMessageModule::CoordinationMessageModule(CoordinationModule * aCoordinationModule, QObject *parent) :
    coordinationModule(aCoordinationModule), QThread(parent)
{
    checkNewMessageInQueueTimer = new QTimer();    
    messageTimeoutMapper = new QSignalMapper();
    connect(messageTimeoutMapper, SIGNAL(mapped(int)), this, SLOT(onSingleMessageTimerExpired(int)));
    connect(checkNewMessageInQueueTimer, SIGNAL(timeout()), this, SLOT(onCheckTimerMessageExpired()));
}

void CoordinationMessageModule::onAddNewMessage(const Data::Message &msg, bool needAck, int resendInterval, int ackCode)
{
    emit sigMessageSendCMM(msg);
    if ((needAck) && (ackCode != 0) && (resendInterval != 0)) {
        QTimer *msgTimer = new QTimer();
        msgTimer->setInterval(resendInterval);
        msgTimer->start();
        int timerId = msgTimer->timerId();
        messages.insert(timerId, ackCode, msg);
        connect(msgTimer, SIGNAL(timeout()), messageTimeoutMapper, SLOT(map()));
        messageTimeoutMapper->setMapping(msgTimer, timerId);
        messageQTimerHash.insert(timerId, msgTimer);
    }else{
        //delete &msg;
    }
}

void CoordinationMessageModule::onAckMessage(const Data::AckMessage &ackMessage)
{    
    int ackCode = ackMessage.getAckId();
    messages.removeBySecondKey(ackCode);
}

void CoordinationMessageModule::onSingleMessageTimerExpired(int timerId)
{    
    if(!messages.containsByFirstKey(timerId)){
        QTimer *timer = messageQTimerHash[timerId];
        timer->stop();
        delete timer;
        messageQTimerHash.remove(timerId);
        //delete &messages.getByFirstKey(timerId);
        //messages.removeByFirstKey(timerId);
    }else{
        emit sigMessageSendCMM(messages.getByFirstKey(timerId));
    }
}

void CoordinationMessageModule::onCheckTimerMessageExpired()
{    
    //ldbg << "checking if message in queue from thread " << this->currentThreadId() << endl;
    if(coordinationMessageQueue->size()==0)
        return;
    const Data::BuddyMessage *buddy = coordinationMessageQueue->dequeue();
    ldbg << "c  un messaggio nella queue di tipo " << buddy->getContent() << endl;
    if(buddy->getContent() == Data::BuddyMessage::AskBidsOnFrontiers){
        coordinationModule->getFrontierAllocationModule()->onAskBidsMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::SendBidsOnFrontiers){
        coordinationModule->getFrontierAllocationModule()->onSendBidsMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::FrontierAssignment){
        coordinationModule->getFrontierAllocationModule()->onAssignItemMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::AskBidsOnVictims){
        //coordinationModule->getVictimAllocationModule()->onAskBidsMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::SendBidsOnVictims){
        //coordinationModule->getVictimAllocationModule()->onSendBidsMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::VictimAssignment){
        //coordinationModule->getVictimAllocationModule()->onAssignItemMessageReceived(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::VictimConfirmation) {
        //coordinationModule->getVictimAllocationModule()->onVictimConfirmationMessage(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::VictimDeletion) {
        //coordinationModule->getVictimAllocationModule()->onDeleteVictimMessage(*buddy);
    }else if(buddy->getContent() == Data::BuddyMessage::AcknowledgmentMessage) {
        onAckMessage(*buddy->get<Data::AckMessage>());
    }else{
        ldbg << "message of unknown type" << endl;
    }
}


void CoordinationMessageModule::start()
{
    ldbg << "starting the thread of the coordinationmessagemodule" << endl;
    checkNewMessageInQueueTimer->start(CHECK_MESSAGE_TIMEOUT);
    QThread::start();
}

void CoordinationMessageModule::run()
{
    exec();
}

void CoordinationMessageModule::setCoordinationMessageQueue(Shared::MutexQueue<const Data::BuddyMessage *> *aCoordinationMessageQueue)
{
    coordinationMessageQueue = aCoordinationMessageQueue;
}

Shared::MutexQueue<const Data::BuddyMessage *> * CoordinationMessageModule::getBidsQueue()
{    
    return coordinationMessageQueue;
}


}
