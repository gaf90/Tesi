#ifndef INVERSEKINEMATIC_H
#define INVERSEKINEMATIC_H

#include "data/pose.h"
#include "wheelspeeds.h"

#define MINIMUM_SPEED 0.2
#define ZERO_THRESHOLD 0.05
#define SAFETY_PERC .75

#define INVERSE_KIN_TRASL_TOL 0.1
#define INVERSE_KIN_ANG_TOL 0.1

class InverseKinematic
{
public:
    InverseKinematic();
    virtual ~InverseKinematic();

    virtual const WheelSpeeds computeSpeeds(const Data::Pose &destPose, double dt) const = 0;
protected:
    double setMinimumSpeed(double speed,bool isKenaf) const;
};

#endif // INVERSEKINEMATIC_H
