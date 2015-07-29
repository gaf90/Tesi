#ifndef COORDINATIONMODULE_H
#define COORDINATIONMODULE_H

#include <QObject>
#include <QHash>
#include <QTimer>
#include <QThread>
#include "coordination/auction/frontierauction.h"
#include "data/message.h"
#include "shared/mutexqueue.h"
#include "data/objecttable.h"
#include "coordinationmessagemodule.h"
#include "coordination/frontierallocationmodule.h"
#include "coordination/victimallocationmodule.h"

namespace Exploration {
    class ExplorationModule;
    class EvaluationRecords;
}


namespace Coordination{
    class CoordinationModule : public QObject
    {
        Q_OBJECT
    public:
        CoordinationModule(int robotId, QObject *parent = 0);
        void setExplorationModule(Exploration::ExplorationModule * expModule);
        void setPathPlannerModule(PathPlanner::PathPlannerModule * pathPlannerModule);
        void setCoordinationMessageQueue(Shared::MutexQueue<const Data::BuddyMessage *> * aCoordinationMessageQueue);
        Shared::MutexQueue<const Data::BuddyMessage *> *getCoordinationMessageQueue();
        ~CoordinationModule();
        void startNewFrontierAuction(Exploration::EvaluationRecords *evaluationRecords = 0, bool forceAssignment = false);
        FrontierAllocationModule * getFrontierAllocationModule();
        VictimAllocationModule * getVictimAllocationModule();
        CoordinationMessageModule* getCoordinationMessageModule();

    signals:        
        void sigMessageSendCM(const Data::Message &msg);
        void sigPointToReachCM(double x, double y);

    public slots:        

    private slots:

    private:
        CoordinationMessageModule *coordinationMessageModule;
        FrontierAllocationModule *frontierAllocationModule;
        VictimAllocationModule *victimAllocationModule;
        int robotId;

    };
}

#include "exploration/explorationmodule.h"
#include "exploration/evaluationrecords.h"

#endif // COORDINATIONMODULE_H
