#ifndef WAYPOINTCOMMAND_H
#define WAYPOINTCOMMAND_H

#include "data/serializable.h"
#include "data/pose.h"
#include "data/message.h"

namespace Data{

/**
* @brief WaypointCommand contains a set of waypoints to be sent to a robot.
*
* The robot will handle the set of points and navigate through them, without
* any free exploration behaviour.
*
* @see
*/
class WaypointCommand : public Serializable, public Message
{
public:
    WaypointCommand();

    /**
    * Creates a WaypointCommand message with the list of waypoints to follow
    *
    * @param waypoints QList<Pose> of the waypoints to follow
    * @param notify bool, if true the robot notifies the base station when the last waypoint of the list is
    * reached. If false the robot could autonomously continue the exploration indipendently from the operator
    * @param timer the time, in seconds, that the robot must wait if notify==true when it finishes the current task
    * before start to freely explore the environment.
    */
    WaypointCommand(Pose waypoint, bool notify, uint timer);

    /**
     * Destroys the WaypointCommand
     */
    virtual ~WaypointCommand();

    /**
    * @return the QList<Pose> that identify the sequence of waypoints to be reached
    */
    Pose getWaypoint() const;

    /**
    * @return a boolean that specifies if the robot must notify the base station when reaches the last waypoint
    */
    bool isNotificationNeeded() const;

    uint getTimer() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    Pose waypoint;
    bool notifyWhenFinished;
    uint timer;

};

}

#endif // WAYPOINTCOMMAND_H
