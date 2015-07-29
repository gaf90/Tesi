#ifndef HIGHLEVELCOMMAND_H
#define HIGHLEVELCOMMAND_H

#include "data/serializable.h"
#include "data/pose.h"
#include "data/message.h"

namespace Data{


/**
 * @brief High command message sent from the Basestation to a robot.
 *
 * The HighLevelCommand class is a container for a High level command sent from the
 * basestation to a single robot. It can specify both a search direction and/or a search area
 *
 *
 * @see
 */
class HighLevelCommand :  public Serializable, public Message
{
public:
    HighLevelCommand();

    /**
    * creates a HighLevelCommand message with the Pose indicating the preferred direction to explore
    *
    * @param pose Pose, it shows the direction to explore (coordinates puls theta angle)
    * @param notify bool, if true the robot notifies the base station when the task is completed.
    * If false the robot could autonomously continue the exploration indipendently from the operator
    * @param timer the time, in seconds, that the robot must wait if notify==true when it finishes the current task
    * before start to freely explore the environment.
    */
    HighLevelCommand(double x, double y, bool notify, uint  waitTime, bool isDirection);

    virtual ~HighLevelCommand();

    /**
    * @return preferred X value to explore.
    */
    double getX() const;

    /**
    * @return preferred Y value to explore.
    */
    double getY() const;

    bool notifyWhenActionFinished() const;

    uint getWaitTime() const;

    bool exploreDirection() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    double x, y;
    bool notifyWhenFinished;
    uint waitTime;
    bool isDirection;



};

}
#endif // HIGHLEVELCOMMAND_H
