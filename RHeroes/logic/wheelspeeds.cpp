#include "wheelspeeds.h"

WheelSpeeds::WheelSpeeds(double rightSpeed, double leftSpeed)
    : leftSpeed(leftSpeed), rightSpeed(rightSpeed)
{
}

WheelSpeeds::~WheelSpeeds()
{

}

double WheelSpeeds::getRightSpeed() const
{
    return rightSpeed;
}
double WheelSpeeds::getLeftSpeed() const
{
    return leftSpeed;
}

void WheelSpeeds::setLeftSpeed(double leftSpeed)
{
    this->leftSpeed = leftSpeed;
}

void WheelSpeeds::setRightSpeed(double rightSpeed)
{
    this->rightSpeed = rightSpeed;
}
