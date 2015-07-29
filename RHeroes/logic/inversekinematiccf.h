#ifndef INVERSEKINEMATICCF_H
#define INVERSEKINEMATICCF_H

#include "inversekinematic.h"

#define RH_TEMP 0.63

class InverseKinematicCF : public InverseKinematic
{
public:
    InverseKinematicCF(double L,bool isKenaf);
    virtual ~InverseKinematicCF();

    virtual const WheelSpeeds computeSpeeds(const Data::Pose &to, double dt) const;

    static const double alpha = 0.2;
private:
    double solveAngularSpeed(const Data::Pose &to, double dt) const;
    double solveRadius(const Data::Pose &to, double w, double dt) const;
    const WheelSpeeds scaleSpeeds(double rightSpeed, double leftSpeed) const;
    double L;
    bool isKenaf;

};

#endif // INVERSEKINEMATICCF_H
