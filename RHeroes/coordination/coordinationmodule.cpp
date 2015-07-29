#include "coordinationmodule.h"
#include "coordinationconstants.h"
#include "data/buddymessage.h"
#include "data/wirelessmessage.h"
#include "shared/utilities.h"
#include "shared/config.h"

#include <typeinfo>
#include <QTimer>
#include <QTimerEvent>

namespace Coordination {
    using namespace Data;

    CoordinationModule::CoordinationModule(int aRobotId, QObject *parent) :
        QObject(parent), robotId(aRobotId)
    {
        coordinationMessageModule = new CoordinationMessageModule(this);
        frontierAllocationModule = new FrontierAllocationModule(robotId);
        victimAllocationModule = new VictimAllocationModule(robotId);
        connect(frontierAllocationModule, SIGNAL(sigAddNewCoordinationMessage(Data::Message,bool, int, int)), coordinationMessageModule, SLOT(onAddNewMessage(Data::Message,bool, int, int)));
        connect(victimAllocationModule, SIGNAL(sigAddNewCoordinationMessage(Data::Message,bool, int, int)), coordinationMessageModule, SLOT(onAddNewMessage(Data::Message,bool, int, int)));
        connect(coordinationMessageModule, SIGNAL(sigMessageSendCMM(Data::Message)), this, SIGNAL(sigMessageSendCM(Data::Message)));
        connect(frontierAllocationModule, SIGNAL(sigFrontierToReachFAM(double, double)), this, SIGNAL(sigPointToReachCM(double,double)));

        coordinationMessageModule->start();
        victimAllocationModule->start();
    }      

    CoordinationModule::~CoordinationModule()
    {
    }

    void CoordinationModule::setExplorationModule(Exploration::ExplorationModule *expModule)
    {
        frontierAllocationModule->setExplorationModule(expModule);
    }

    void CoordinationModule::setPathPlannerModule(PathPlanner::PathPlannerModule *pathPlannerModule)
    {
        victimAllocationModule->setPathPlannerModule(pathPlannerModule);
    }

    void CoordinationModule::setCoordinationMessageQueue(Shared::MutexQueue<const Data::BuddyMessage *> * aCoordinationMessageQueue)
    {
        coordinationMessageModule->setCoordinationMessageQueue(aCoordinationMessageQueue);
    }

    Shared::MutexQueue<const Data::BuddyMessage *> * CoordinationModule::getCoordinationMessageQueue()
    {
        return coordinationMessageModule->getBidsQueue();
    }        

    void CoordinationModule::startNewFrontierAuction(Exploration::EvaluationRecords *evaluationRecords, bool forceAssignment)
    {
        frontierAllocationModule->startNewAuction(evaluationRecords, forceAssignment);
    }

    Coordination::FrontierAllocationModule * CoordinationModule::getFrontierAllocationModule()
    {
        return frontierAllocationModule;
    }

    Coordination::CoordinationMessageModule* CoordinationModule:: getCoordinationMessageModule(){
        return coordinationMessageModule;
    }

    Coordination::VictimAllocationModule * CoordinationModule::getVictimAllocationModule()
    {
        return victimAllocationModule;
    }
}
