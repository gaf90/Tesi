#include "waypointcommand.h"



namespace Data{

WaypointCommand::WaypointCommand():
    waypoint(Pose()),
    notifyWhenFinished(true),
    timer(0)
{
}

WaypointCommand::WaypointCommand(Pose waypoint, bool notify, uint timer):
    waypoint(waypoint),
    notifyWhenFinished(notify),
    timer(timer)
{
}

WaypointCommand::~WaypointCommand()
{
}

Pose WaypointCommand::getWaypoint() const
{
    return waypoint;
}

bool WaypointCommand::isNotificationNeeded() const
{
    return notifyWhenFinished;
}

void WaypointCommand::serializeTo(QDataStream &stream) const
{
    stream << waypoint << notifyWhenFinished;
}

void WaypointCommand::deserializeFrom(QDataStream &stream)
{
    stream >> waypoint >> notifyWhenFinished;
}

uint WaypointCommand::getTimer() const
{
    return timer;
}

}
