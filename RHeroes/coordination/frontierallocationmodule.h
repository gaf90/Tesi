#ifndef FRONTIERALLOCATIONMODULE_H
#define FRONTIERALLOCATIONMODULE_H

#include "coordination/allocationmodule.h"
#include "coordination/auction/frontierauction.h"

namespace Exploration {
    class ExplorationModule;
}

namespace Coordination{
    class FrontierAllocationModule : public AllocationModule
    {
    public:
        FrontierAllocationModule(int robotId);
        ~FrontierAllocationModule();
        void setExplorationModule(Exploration::ExplorationModule * expModule);
        void startNewAuction(Exploration::EvaluationRecords *evaluationRecords, bool forceAssignment = false);
        void onAskBidsMessageReceived(const Data::BuddyMessage &msg);
        void onSendBidsMessageReceived(const Data::BuddyMessage &msg);
        void onAssignItemMessageReceived(const Data::BuddyMessage &msg);


    signals:

    public slots:

    private slots:
        void onAuctionExpired();

    private:
        bool existInterestingFrontier(QHash<uint, double> evaluations);
        QHash<uint, double> getInterestingFrontiers(QHash<uint, double> initialEvaluations);
        void assignFrontier(SLAM::Geometry::Frontier const &frontier, double frontierEvaluation);

        Exploration::ExplorationModule *explorationModule;
        FrontierAuction *auction;
        Data::Pose lastFrontierAssigned; //identified by the centroid
        double lastFrontierAssignedEvaluation;
        bool forceAssignment; //true if whichever assigned frontier must be accepted
        bool firstAssignment;


        bool eliminami_ServoSoloPerProblemiAlPathPlanner;
    };
}

#include "exploration/explorationmodule.h"

#endif // FRONTIERALLOCATIONMODULE_H
