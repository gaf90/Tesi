#include "victimallocationmodule.h"
#include "data/robotvictimcouplingmessage.h"

//delete when victim module is working
#include "math.h"
#include "time.h"

//remove when implemented the victim detection module, or something similar
#define START_RANDOM_VICTIM_CREATION 10000
#define RANDOM_VICTIM_CREATION_PROBABILITY 35 //if the number randomly extracted is below this percentage than a victim is created
#define MAX_VICTIM_CREATED 5
//#define RANDOM_VICTIM_ON

namespace Coordination{
    VictimAllocationModule::VictimAllocationModule(int robotId)
        : AllocationModule(robotId), auction(new VictimAuction()),
          isAuctionRequiredTimer(new QTimer()), goCloseToAVictimTimer(new QTimer())
    {
        isFinalVictimAssigned = false;
        isAuctionRequiredTimer->setInterval(CHECK_AUCTION_REQUIRED_INTERVAL*fRand(0.75, 1.25)); //scale factor needed to avoid various auctions to happen in the same moment

        connect(isAuctionRequiredTimer, SIGNAL(timeout()), this, SLOT(isAuctionRequired()));

        goCloseToAVictimTimer->setInterval(CHECK_GO_CLOSE_TO_A_VICTIM_REQUIRED_INTERVAL);
        connect(goCloseToAVictimTimer, SIGNAL(timeout()), this, SLOT(isGoCloseToAVictimRequired()));        

        connect(auction, SIGNAL(auctionExpired()), this, SLOT(onAuctionExpired()));


        //remove when implemented the victim detection module, or something similar
        createRandomVictimTimer = new QTimer();
        createRandomVictimTimer->setInterval(START_RANDOM_VICTIM_CREATION);
#ifdef RANDOM_VICTIM_ON
        createRandomVictimTimer->start();
#endif
        connect(createRandomVictimTimer, SIGNAL(timeout()), this, SLOT(createRandomVictim()));
        srand(time(NULL));
        createdVictim = 0;
    }

    VictimAllocationModule::~VictimAllocationModule()
    {
    }

    void VictimAllocationModule::setPathPlannerModule(PathPlanner::PathPlannerModule *aPathPlanningModule)
    {
        pathPlanningModule = aPathPlanningModule;
    }

    void VictimAllocationModule::addNewVictim(uint victim)
    {
        //if victim is in the list of the deleted victim (deleted by the human operator), then it is not a victim
        if(lstDeletedVictims.contains(victim))
            return;
        //if victim already in the list of assigned victim then it is useless, help with problem of unreliable connectivity
        if(!lstAssignedVictims.contains(victim) /*&& !isFinalsigVictimAssigned*/){ //discuss with Calt about this, if I already have my final victim assigned, do i have to accept new victims from the base station?
            if(auction->isAuctionInProgress()){
                victimArrivedDuringAuction.enqueue(victim);
                ldbg << "victim" << victim << " arrived during auction, put in queue" << endl;
            } else {
                ldbg << "adding victim " << victim << endl;
                lstAssignedVictims.append(victim);
            }
        } else {
            ldbg << "tried to add victim " << victim << ", it is already mine, operation blocked" << endl;
        }
    }

    void VictimAllocationModule::isAuctionRequired()
    {
        //ldbg << "checking if victim auction required from thread " << this->currentThreadId() << endl;
        isAuctionRequiredTimer->setInterval(CHECK_AUCTION_REQUIRED_INTERVAL*fRand(0.75, 1.25)); //scale factor need to avoid various auctions to happen in the same moment
        if((lstAssignedVictims.size() > 1 && !isFinalVictimAssigned) ||
                (lstAssignedVictims.size() > 0 && isFinalVictimAssigned))
            startNewAuction();
    }

    void VictimAllocationModule::startNewAuction()
    {
        if(auction->isAuctionInProgress())
            auction->endAuction();
        auction->startNewAuction();
        auction->addItems(&lstAssignedVictims);
        if(!isFinalVictimAssigned){
            //if isFinalsigVictimAssigned == true then it has no sense to add my personal bids, I can't win other victims
            QHash<uint, double> evaluations;
            foreach(uint tempVictim, lstAssignedVictims)
                evaluations.insert(tempVictim, fRand(10,50));
            auction->addBids(robotId, evaluations);
        }
        emit sigAddNewCoordinationMessage(auction->createAskBidsMessage(robotId), false);
        ldbg << "starting auction with id :" << auction->getAuctionId() << " on victims:";
        foreach(uint tempVictim, lstAssignedVictims)
            ldbg << " " << tempVictim;
        ldbg << endl;
    }

    void VictimAllocationModule::onAskBidsMessageReceived(const Data::BuddyMessage &msg)
    {
        ldbg << "received request for joining frontier auction" << endl;
        if(auction->isAuctionInProgress())
            return; //i am already managing an auction, it is risky to merge different auctions, may lead to inconsistence
        if(isFinalVictimAssigned)
            return; //i already have a finalVictim coupled with me, i can't win new victims
        const AskBidsMessage<uint> *askBidsMessage = auction->parseAskBidsMessage(msg);
        QList<uint> * victims = askBidsMessage->getItems();

        QHash<uint, double> bids;
        foreach(uint tempVictim, *victims){
            if (lstAssignedVictims.contains(tempVictim)){
                // two or more robots possess the same victim, then at the end of the auction at maximum one will is going to have it assigned
                lstAssignedVictims.removeAll(tempVictim);
                ldbg << "i already have a victim with id " << tempVictim << ", i delete it from my list (discovered when parsing askbidsmessage" << endl;
            }
            bids.insert(tempVictim, fRand(10,50));
        }
        WirelessMessage message = auction->createSendBidsMessage(askBidsMessage->getAuctionId(), robotId, bids, askBidsMessage->getSenderId());
        emit sigAddNewCoordinationMessage(message, false);
    }

    void VictimAllocationModule::onSendBidsMessageReceived(const Data::BuddyMessage &msg)
    {
        ldbg << "received bids" << endl;
        if (!auction->isAuctionInProgress())
            //no auction running, probable message related to previous auction, useless
            return;
        const SendBidsMessage *sendBidsMessage = auction->parseSendBidsMessage(msg);
        if(auction->getAuctionId() != sendBidsMessage->getAuctionId())
            //old auction, useless
            return;
        auction->addBids(sendBidsMessage->getBidderId(), *sendBidsMessage->getBids());
    }

    void VictimAllocationModule::onAssignItemMessageReceived(const Data::BuddyMessage &msg)
    {        
        const AssignItemMessage<uint> * assignVictimMessage = auction->parseAssignItemMessage(msg);
        const uint victim = assignVictimMessage->getItem();
        addNewVictim(victim);
        ldbg << "robot with id " << msg.getSource() << " assigned me victim " << victim << endl;
        emit sigAddNewCoordinationMessage(auction->createAckMessage(robotId, robotIndexFromName(msg.getSource()), assignVictimMessage->getAckCode()),false);
    }

    void VictimAllocationModule::onAuctionExpired()
    {
        ldbg << "auction expired, starting assignment phase" << endl;
        if (!auction->isAuctionInProgress()){
            ldbg << "auction not in progress" << endl;
            return;
        }
        QHash<int, uint> tempAssignedVictims = auction->assignItems();        
        QHashIterator<int, uint> i(tempAssignedVictims);
        int destinationRobotId;
        uint tempVictim;
        while (i.hasNext()) {
            i.next();
            if(i.key() != robotId){
                int ackCode = rand();
                destinationRobotId = i.key();
                tempVictim = i.value();
                emit sigAddNewCoordinationMessage(auction->createAssignItemMessage(tempVictim, robotId, destinationRobotId, ackCode), true, VICTIM_RESEND_FOR_ACK_INTERVAL, ackCode);
                lstAssignedVictims.removeAll(i.value());
                ldbg << "assigned victim " << i.value() << " to robot " << i.key() << endl;
            }else{
                ldbg << "assigned victim " << i.value() << "  to myself" << endl;
            }
        }
        auction->endAuction();
        ldbg << "auction completed" << endl << endl;
        if (victimArrivedDuringAuction.size() > 0)
            manageVictimArrivedDuringAuction();
    }

    void VictimAllocationModule::onVictimConfirmationMessage(const Data::BuddyMessage &msg)
    {
        ldbg << "appena chiamata la funzione per gestire il mex di calt" << endl;
        const Data::VictimConfirmationMessage *victimConfirmationMessage = msg.get<Data::VictimConfirmationMessage>();
        if(victimConfirmationMessage->isNewVictim() == false)
            return; //just a change of the position of the victim made by the human operator
        ldbg << "messaggio parsato, victimmessage is " << victimConfirmationMessage << endl;
        bool isNew = victimConfirmationMessage->isNewVictim();
        if(isNew){
            uint victim = victimConfirmationMessage->getVictimID();
            ldbg << "calt is asking me to add victim" << victim << endl;
            addNewVictim(victim);
            ldbg << "basestation assigned me victim " << victim << endl;
            //no ack 'cause not asked, check later
            //delete VictimConfirmationMessage;
        }
    }

    void VictimAllocationModule::onDeleteVictimMessage(const Data::BuddyMessage &msg)
    {
        const Data::VictimDelectionMessage *victimDeletionMessage = msg.get<Data::VictimDelectionMessage>();
        if(lstAssignedVictims.contains(victimDeletionMessage->getVictimID())){
            if(auction->isAuctionInProgress()){
                auction->endAuction();
            }
            lstAssignedVictims.removeAll(victimDeletionMessage->getVictimID());
        }
        lstDeletedVictims.append(victimDeletionMessage->getVictimID());
        //delete victimDeletionMessage;
        ldbg << "vittima eliminata numero: " << victimDeletionMessage->getVictimID() << endl;
    }


    void VictimAllocationModule::manageVictimArrivedDuringAuction()
    {
        while(victimArrivedDuringAuction.size() > 0)
            addNewVictim(victimArrivedDuringAuction.dequeue());
    }


    void VictimAllocationModule::isGoCloseToAVictimRequired()
    {
        if(lstAssignedVictims.size() > 0){
            ldbg << "victims i am owning:";
            foreach(uint tempVictim, lstAssignedVictims)
                ldbg << " " << tempVictim;
            ldbg << endl;
        }

        int timeToReachVictim;
        int remainingTime = rand();
        foreach(uint tempVictim, lstAssignedVictims){
            timeToReachVictim = rand();
            //send message to base station comunicating time to reach pose (and in an implicit way also the coupling robot-victim)
            Data::RobotVictimCouplingMessage *robotVictimCouplingMessage = new RobotVictimCouplingMessage(tempVictim, timeToReachVictim);
            Data::BuddyMessage *buddy = new Data::BuddyMessage(robotNameFromIndex(robotId), robotVictimCouplingMessage);
            ldbg << "i am robot" << robotId <<" and to reach victim" << tempVictim << " i need " << timeToReachVictim << "potatoes" << endl;
            Data::WirelessMessage message(buddy, Data::WirelessMessage::MessageExchange);
            emit sigAddNewCoordinationMessage(message);
            if(((remainingTime/timeToReachVictim) <  0/*GO_TO_VICTIM_THRESHOLD*/) && //################################
                    (remainingTime < TIME_TO_GO_TO_VICTIM_THRESHOLD)){
                ldbg << "yeah, it is time to park close to a victim" << endl;
                isFinalVictimAssigned = true;
                lstAssignedVictims.removeAll(tempVictim);
                finalVictim = tempVictim;
                //call something to force movement on "finalVictim"
                goCloseToAVictimTimer->stop();
                break;
                //TODO sooner or later add a message sent every X seconds that notify victim assigned to purge other robots' list
            }
        }
    }

    void VictimAllocationModule::start()
    {
        isAuctionRequiredTimer->start();
        goCloseToAVictimTimer->start();
        QThread::start();
    }

    //remove when implemented the victim detection module, or something similar
    void VictimAllocationModule::createRandomVictim()
    {        
        int randomInt = (int)fRand(1,100);     
        if ((randomInt < RANDOM_VICTIM_CREATION_PROBABILITY)){
            uint randomVictim = (uint) fRand(1,5);
            addNewVictim(randomVictim);
            createdVictim++;
            if(createdVictim > MAX_VICTIM_CREATED){
                createRandomVictimTimer->stop();
            }

        }
    }

    double VictimAllocationModule::fRand(double fMin, double fMax)
    {
        double f = (double)rand() / RAND_MAX;
        return fMin + f * (fMax - fMin);
    }
}
