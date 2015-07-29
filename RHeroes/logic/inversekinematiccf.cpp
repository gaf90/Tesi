#include "inversekinematiccf.h"
#include "shared/utilities.h"
#include "shared/config.h"

InverseKinematicCF::InverseKinematicCF(double L,bool aisKenaf)
    : isKenaf(aisKenaf),L(L)
{
}

InverseKinematicCF::~InverseKinematicCF()
{

}

const WheelSpeeds InverseKinematicCF::computeSpeeds(const Data::Pose &to, double dt) const
{
    const double w = solveAngularSpeed(to, dt);
    if(almostEqual(w, 0, 1e-3)) {
        double v = (to.y() < 0 ? -1 : 1) * std::sqrt(to.x() * to.x() + to.y() * to.y()) / dt;
        v /= RH_TEMP;

        return scaleSpeeds(v,v);
        //return WheelSpeeds(setMinimumSpeed(v), setMinimumSpeed(v));
        //return WheelSpeeds(v,v);
    } else {
        const double R = solveRadius(to, w, dt);
        double rs = w * (R + .5 * L), ls = w * (R - .5 * L);
        rs /= RH_TEMP; ls /= RH_TEMP;

        return scaleSpeeds(rs, ls);
        //return WheelSpeeds(setMinimumSpeed(rs), setMinimumSpeed(ls));
        //return WheelSpeeds(rs, ls);
    }
}

double InverseKinematicCF::solveAngularSpeed(const Data::Pose &to, double dt) const
{
    const double xt = to.x(), yt = to.y(), thetat = to.theta();
    double w = thetat / dt, previous = INFINITY;
    int cnt = 0;

    while(std::fabs(w - previous) > 1e-3 && cnt++ < 20) {
        const double c = std::cos(w * dt), s = std::sin(w * dt);
        const double tmp0 = 2 * xt * yt, tmp1 = (xt - yt) * (xt + yt);
        previous = w;
        w = w + (4 * (thetat - dt * w) * alpha - c * tmp0 + s * tmp1) /
                (dt * (4 * alpha - c * tmp1 - s * tmp0));

    }
    return w;

}

double InverseKinematicCF::solveRadius(const Data::Pose &to, double w, double dt) const
{
    const double xt = to.x(), yt = to.y();
    return .5 * (yt / std::tan(.5 * dt * w) - xt);
}

const WheelSpeeds InverseKinematicCF::scaleSpeeds(double rightSpeed, double leftSpeed) const
{
    double newRightSpeed = rightSpeed;
    double newLeftSpeed = leftSpeed;

    if(leftSpeed >= MAX_SPEED_RH || rightSpeed >= MAX_SPEED_RH){
        if(leftSpeed > rightSpeed){
            newRightSpeed = MAX_SPEED_RH*rightSpeed/leftSpeed;
            newLeftSpeed = MAX_SPEED_RH;
        } else if (leftSpeed < rightSpeed){
            newRightSpeed = MAX_SPEED_RH;
            newLeftSpeed = MAX_SPEED_RH*leftSpeed/rightSpeed;
        } else {
            newRightSpeed = MAX_SPEED_RH;
            newLeftSpeed = MAX_SPEED_RH;
        }
    }
    return WheelSpeeds(setMinimumSpeed(newRightSpeed,isKenaf), setMinimumSpeed(newLeftSpeed,isKenaf));
}
