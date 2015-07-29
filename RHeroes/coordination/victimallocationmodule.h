#ifndef VICTIMALLOCATIONMODULE_H
#define VICTIMALLOCATIONMODULE_H

#include "coordination/allocationmodule.h"
#include "coordination/auction/victimauction.h"
#include "shared/mutexqueue.h"
#include "pathPlanner/pathplannermodule.h"

/*#include <QThread>*/

namespace Coordination{
    class VictimAllocationModule : public AllocationModule
    {
        Q_OBJECT
    public:
        VictimAllocationModule(int robotId);
        ~VictimAllocationModule();
        void setPathPlannerModule(PathPlanner::PathPlannerModule *pathPlanningModule);
        void startNewAuction();
        void addNewVictim(uint victim);
        void onAskBidsMessageReceived(const Data::BuddyMessage &msg);
        void onSendBidsMessageReceived(const Data::BuddyMessage &msg);
        void onAssignItemMessageReceived(const Data::BuddyMessage &msg);
        void onDeleteVictimMessage(const Data::BuddyMessage &msg);
        void onVictimConfirmationMessage(const Data::BuddyMessage &msg);

    signals:

    public slots:
        virtual void start(); //starting the thread

    private slots:
        void onAuctionExpired();
        void isAuctionRequired();
        void isGoCloseToAVictimRequired();


        //remove when implemented the victim detection module, or something similar
        void createRandomVictim();

    protected:
        /*void run();*/

    private:
        void manageVictimArrivedDuringAuction();
        /*
         * a robot may have a maximum of 1 victim assigned, however due to the need of the validation from the base station
         * it is possible that it has more than 1, so, a QList is required. Obviously, it is mandatory to return to the standard
         * situation selling the exceeding victims
         */
        PathPlanner::PathPlannerModule *pathPlanningModule;
        QList<uint> lstAssignedVictims;
        QList<uint> lstDeletedVictims;
        VictimAuction *auction;
        Shared::MutexQueue<uint> victimArrivedDuringAuction;
        QTimer *isAuctionRequiredTimer;
        QTimer *goCloseToAVictimTimer;
        //remove when implemented the victim detection module, or something similar
        QTimer *createRandomVictimTimer;
        uint finalVictim;
        bool isFinalVictimAssigned;
        int createdVictim;


        //remove when implemented the victim detection module, or something similar
        double fRand(double fMin, double fMax);
    };
}

#endif // VICTIMALLOCATIONMODULE_H
