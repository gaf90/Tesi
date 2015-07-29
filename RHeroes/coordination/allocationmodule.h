#ifndef ALLOCATIONMODULE_H
#define ALLOCATIONMODULE_H

#include <QThread>
#include <QTimer>
#include <QObject>
#include "data/wirelessmessage.h"
#include "data/askbidsmessage.h"
#include "data/sendbidsmessage.h"
#include "data/assignitemmessage.h"
#include "coordination/auction/auction.h"
#include "data/destinationmessage.h"

namespace Coordination{
    class AllocationModule : public QThread
    {
    Q_OBJECT
    public:
        explicit AllocationModule(int aRobotId, QObject *parent = 0);
        virtual ~AllocationModule();
        virtual void onAskBidsMessageReceived(const Data::BuddyMessage &msg) = 0;
        virtual void onSendBidsMessageReceived(const Data::BuddyMessage &msg) = 0;
        virtual void onAssignItemMessageReceived(const Data::BuddyMessage &msg) = 0;
        Data::WirelessMessage createDestinationMessage(SLAM::Geometry::Point point);

    signals:
        void sigAddNewCoordinationMessage(const Data::Message &msg, bool needAck = false, int resendInterval = 0, int ackCode = 0);
        void sigFrontierToReachFAM(double x, double y);

    public slots:

    private slots:
        virtual void onAuctionExpired() = 0;

    protected:
        void run();
        int robotId;
    };
}
#endif // ALLOCATIONMODULE_H
