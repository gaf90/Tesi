#ifndef WHEELSPEEDS_H
#define WHEELSPEEDS_H

class WheelSpeeds
{
public:
    WheelSpeeds(double rightSpeed, double leftSpeed);
    virtual ~WheelSpeeds();

    double getRightSpeed() const;
    double getLeftSpeed() const;

    void setLeftSpeed(double leftSpeed);
    void setRightSpeed(double rightSpeed);


private:
    double leftSpeed;
    double rightSpeed;
};

#endif // WHEELSPEEDS_H
