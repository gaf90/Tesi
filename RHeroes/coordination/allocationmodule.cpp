#include "coordination/allocationmodule.h"

namespace Coordination{
AllocationModule::AllocationModule(int aRobotId, QObject *parent)
    : robotId(aRobotId), QThread(parent)
{
    //connect(auctionTimer, SIGNAL(timeout()), this, SLOT(onAuctionExpired()));
}


AllocationModule::~AllocationModule()
{
}

void AllocationModule::run()
{
    exec();
}

Data::WirelessMessage AllocationModule::createDestinationMessage(SLAM::Geometry::Point point)
{
    DestinationMessage *destinationMessage = new DestinationMessage(robotId, point);
    BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(robotId), destinationMessage);
    Data::WirelessMessage message(buddy, WirelessMessage::MessageExchange);
    return message;
}
}

