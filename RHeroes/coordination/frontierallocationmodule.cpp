#include "frontierallocationmodule.h"
#include "coordination/coordinationconstants.h"
#include "coordination/auction/frontierauction.h"


namespace Coordination{
FrontierAllocationModule::FrontierAllocationModule(int robotId)
    : AllocationModule(robotId), auction(new FrontierAuction())
{
    connect(auction, SIGNAL(auctionExpired()), this, SLOT(onAuctionExpired()));
    firstAssignment = true;
    eliminami_ServoSoloPerProblemiAlPathPlanner = false;
}

FrontierAllocationModule::~FrontierAllocationModule()
{
}

void FrontierAllocationModule::setExplorationModule(Exploration::ExplorationModule *expModule)
{
    explorationModule = expModule;
}

void FrontierAllocationModule::startNewAuction(Exploration::EvaluationRecords *evaluationRecords, bool aForceAssignment)
{
    ldbg << "Frontier Allocation Module: Start new Auction. Force Assignment?" << aForceAssignment << " on frontiers:";

    foreach(SLAM::Geometry::Frontier frontier,  *(evaluationRecords->getEvaluatedFrontiers())){
        ldbg << " " << frontier.centroid();
    }
    ldbg << endl;

    forceAssignment = aForceAssignment;
    if(auction->isAuctionInProgress()){
        ldbg << "Frontier Allocation Module: Stopping old auction still running" << endl;
        auction->endAuction();
    }

    if(existInterestingFrontier(evaluationRecords->getEvaluationFrontiersBids()) || forceAssignment)
    {
        auction->startNewAuction();
        QHash<uint, double> evaluationBids;
        if(forceAssignment || firstAssignment)
        {
            ldbg << "Frontier Allocation Module: Consider all the bids." << endl;
            evaluationBids = evaluationRecords->getEvaluationFrontiersBids();
        }
        else
        {
            ldbg << "Frontier Allocation Module: Consider only interesting bids." << endl;
            evaluationBids = getInterestingFrontiers(evaluationRecords->getEvaluationFrontiersBids());
        }

        ldbg << "Frontier Allocation Module: Frontiers and evaluations are:" << endl;
        foreach(uint frontier, evaluationBids.keys())
            ldbg << " (" << frontier << ", " << evaluationBids[frontier] << ")";
        ldbg << endl;

        auction->addItems(evaluationRecords->getEvaluatedFrontiers());
        auction->addBids(robotId, evaluationBids);

    }
}

void FrontierAllocationModule::onAskBidsMessageReceived(const Data::BuddyMessage &msg)
{
    const AskBidsMessage<SLAM::Geometry::Frontier> *askBidsMessage = auction->parseAskBidsMessage(msg);
    QList<SLAM::Geometry::Frontier> * frontiers = askBidsMessage->getItems();
    QList<SLAM::Geometry::Frontier *> *newFrontiers = new QList<SLAM::Geometry::Frontier *>();

    foreach(SLAM::Geometry::Frontier f, *frontiers){
        newFrontiers->append(new SLAM::Geometry::Frontier(f));
    }

    Exploration::EvaluationRecords* evaluationRecords = explorationModule->evaluateFrontiers(*newFrontiers);

    ldbg << "Frontier Allocation Module: New frontier evaluated!" << endl;

    QHash<uint, double> evaluationBids = evaluationRecords->getEvaluationFrontiersBids();

    if(existInterestingFrontier(evaluationBids))
    {
        QHash<uint, double> finalBids;
        finalBids = getInterestingFrontiers(evaluationBids);
        WirelessMessage message = auction->createSendBidsMessage(askBidsMessage->getAuctionId(), robotId, finalBids, askBidsMessage->getSenderId());
        emit sigAddNewCoordinationMessage(message, false);
    }

    for(int i=newFrontiers->length()-1; i>=0; i--)
    {
        delete newFrontiers->at(i);
        newFrontiers->removeAt(i);
    }

    delete newFrontiers;
}

void FrontierAllocationModule::onSendBidsMessageReceived(const Data::BuddyMessage &msg)
{
    if (!auction->isAuctionInProgress())
        ldbg << "Frontier Allocation: Received bid message, but no auction is running. Discard it."<<endl;
        return;
    const SendBidsMessage *sendBidsMessage = auction->parseSendBidsMessage(msg);
    if(auction->getAuctionId() != sendBidsMessage->getAuctionId())
        ldbg << "Frontier Allocation: Received bid message, but belongs to a previous auction. Discard it."<<endl;
        return;
    auction->addBids(sendBidsMessage->getBidderId(), *sendBidsMessage->getBids());
}

void FrontierAllocationModule::onAssignItemMessageReceived(const Data::BuddyMessage &msg)
{
    ldbg << "Frontier Allocation: Robot with id " << msg.getSource() << " assigned me a frontier" << endl;
    const AssignItemMessage<SLAM::Geometry::Frontier> * assignFrontierMessage = auction->parseAssignItemMessage(msg);
    const SLAM::Geometry::Frontier frontier = assignFrontierMessage->getItem();
    assignFrontier(frontier, assignFrontierMessage->getEvaluation());
}

void FrontierAllocationModule::onAuctionExpired()
{
    if (!auction->isAuctionInProgress())
        return;
    QHash<int, SLAM::Geometry::Frontier> assignedFrontiers = auction->assignItems();
    SLAM::Geometry::Frontier frontier;
    double frontierEvaluation;
    if(assignedFrontiers.contains(robotId)){
        frontier = assignedFrontiers[robotId];
        frontierEvaluation = auction->getEvaluation(robotId, qHash(frontier));
        if(frontierEvaluation != ITEM_NOT_INTERESTING){
            assignFrontier(frontier, frontierEvaluation);
            ldbg << "Frontier Allocation Module: Assign frontier " << qHash(frontier) << " to myself" << endl;
        }else{
            ldbg << "Frontier Allocation Module: Frontier received is not interesting." << endl;
        }
        assignedFrontiers.remove(robotId);
    }
    int robotIdKey;
    foreach(robotIdKey, assignedFrontiers.keys()){
        frontier = assignedFrontiers[robotIdKey];
        frontierEvaluation = auction->getEvaluation(robotIdKey, qHash(frontier));
        if(frontierEvaluation != ITEM_NOT_INTERESTING)
            emit sigAddNewCoordinationMessage(auction->createAssignItemMessage(frontier, robotId, robotIdKey, frontierEvaluation), false);
        ldbg << "Frontier Allocation Module: Assigned frontier " << qHash(frontier)<< "to robot " << robotIdKey << endl;
    }
    auction->endAuction();
    forceAssignment = false;
    ldbg << "Frontier Allocation Module: Auction completed" << endl;
}

void FrontierAllocationModule::assignFrontier(const SLAM::Geometry::Frontier &frontier, double frontierEvaluation)
{
    Data::Pose frontierPose(frontier.centroid().x(), frontier.centroid().y(), 0);
    bool assignFrontierFlag = false;
    if(firstAssignment)
    {
        ldbg << "Frontier Allocation Module: Is first assignment!" << endl;
        assignFrontierFlag = true;
        firstAssignment = false;
        lastFrontierAssignedEvaluation = frontierEvaluation;
    }

    ldbg << "Frontier Allocation Module: Actual frontier evaluation " << frontierEvaluation <<
            " , last frontier evaluation " << lastFrontierAssignedEvaluation <<
            ", ratio is " << frontierEvaluation/lastFrontierAssignedEvaluation << endl;


    if(frontierEvaluation/lastFrontierAssignedEvaluation > (1+MINIMUM_EVALUATION_INCREASE))
        assignFrontierFlag = true;
    if ((assignFrontierFlag && (lastFrontierAssigned.getDistance(frontierPose) > MINIMUM_FRONTIER_DISTANCE_FROM_PREVIOUS)) || forceAssignment)
    {
        lastFrontierAssigned = frontierPose;
        lastFrontierAssignedEvaluation = frontierEvaluation;

        emit sigFrontierToReachFAM(frontier.centroid().x(), frontier.centroid().y());        
        emit sigAddNewCoordinationMessage(createDestinationMessage(frontier.centroid()));

        ldbg << "Frontier Allocation Module: Forwarding pose (" << frontierPose.getX() << ", " << frontierPose.getY() << ")" << endl;
    }
}

bool FrontierAllocationModule::existInterestingFrontier(QHash<uint, double> evaluations)
{
    if(firstAssignment)
        return true;
    foreach(double eval, evaluations.values()){
        if(eval/lastFrontierAssignedEvaluation > (1+MINIMUM_EVALUATION_INCREASE))
            return true;
    }
    return false;
}

QHash<uint, double> FrontierAllocationModule::getInterestingFrontiers(QHash<uint, double> initialEvaluations)
{
    if(firstAssignment)
        return initialEvaluations;

    QHash<uint, double> finalEvaluations;
    QHashIterator<uint, double> i(initialEvaluations);

    while(i.hasNext()){
        i.next();
        finalEvaluations.insert(i.key(), (i.value()/lastFrontierAssignedEvaluation > (1+MINIMUM_EVALUATION_INCREASE))
                                ? i.value() : ITEM_NOT_INTERESTING);
    }
    return finalEvaluations;
}

}
