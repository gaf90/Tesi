#include "action.h"
#include "shared/constants.h"
#include "shared/config.h"
#include "cmath"

namespace Data{
Action::Action(ActionType aType, double aValue) :
    AbstractAction(), type(aType), value(aValue)
{
}

Action::Action() : AbstractAction()
{
}

Action::~Action()
{

}

Action::ActionType Action::getType() const
{
   return type;
}

double Action::getValue() const
{
    return value;
}

void Action::setType(Action::ActionType aType)
{
    type = aType;
}

void Action::setValue(double aValue)
{
    value = aValue;
}

double Action::getTimeEstimate()
{
    double toRet = 0;
    if(type == Action::Translation){
        //s = v*t ==> t=s/v
        toRet = value/MED_SPEED;
    }
    //PRM
    else if (type == Action::Rotation){
    //
        //sperimentally, with a wheel speed of 0.2
        //the robot takes 9 seconds to perform 90°
        //so when wheelSpeed = 0.2, it rotates at
        // 10 degree/s. let be speed = 10.
        //th = speed*time =>
        // t = th/speed
        //Notice that 0.2 is the lowest speed we apply to
        //the robot, so this is a worst case estimate
        toRet = value/ROTATION_SPEED_ESTIMATE_OPOINT2;
    }
    return fabs(toRet);
}
}
