#include "inversekinematic.h"

InverseKinematic::InverseKinematic()
{
}

InverseKinematic::~InverseKinematic()
{

}

double InverseKinematic::setMinimumSpeed(double speed, bool isKenaf) const
{
    if (isKenaf)
    {
        double minimum = 0.8;
        if(-minimum < speed && speed < -ZERO_THRESHOLD)
            return -minimum;
        if(-ZERO_THRESHOLD <= speed && speed < 0)
            return 0;
        if(0 <= speed && speed <= ZERO_THRESHOLD)
            return 0;
        if(ZERO_THRESHOLD < speed && speed <= MINIMUM_SPEED)
            return minimum;
    }
    else{
        if(-MINIMUM_SPEED < speed && speed < -ZERO_THRESHOLD)
            return -MINIMUM_SPEED;
        if(-ZERO_THRESHOLD <= speed && speed < 0)
            return 0;
        if(0 <= speed && speed <= ZERO_THRESHOLD)
            return 0;
        if(ZERO_THRESHOLD < speed && speed <= MINIMUM_SPEED)
            return MINIMUM_SPEED;
    }
    return speed;
}
