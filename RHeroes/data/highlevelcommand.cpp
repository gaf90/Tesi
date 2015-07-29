#include "highlevelcommand.h"


namespace Data{

HighLevelCommand::HighLevelCommand():
    x(0), y(0), notifyWhenFinished(false), waitTime(0), isDirection(false)
{
}

HighLevelCommand::HighLevelCommand(double x, double y, bool notify, uint  waitTime, bool isDirection):
    x(x), y(y), notifyWhenFinished(notify), waitTime(waitTime), isDirection(isDirection)
{
}

HighLevelCommand::~HighLevelCommand()
{
}

void HighLevelCommand::serializeTo(QDataStream &stream) const
{
    stream << x << y << notifyWhenFinished << waitTime << isDirection;
}

void HighLevelCommand::deserializeFrom(QDataStream &stream)
{
    stream >> x >> y >> notifyWhenFinished >> waitTime >> isDirection;
}

bool HighLevelCommand::exploreDirection() const
{
    return isDirection;
}

bool HighLevelCommand::notifyWhenActionFinished() const
{
    return notifyWhenFinished;
}

uint HighLevelCommand::getWaitTime() const
{
    return waitTime;
}

double HighLevelCommand::getY() const
{
    return y;
}

double HighLevelCommand::getX() const
{
    return x;
}

}
