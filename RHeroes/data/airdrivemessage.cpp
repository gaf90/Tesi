#include "airdrivemessage.h"

namespace Data{

AirDriveMessage::AirDriveMessage():
    altitudeSpeed(0), linearSpeed(0), lateralSpeed(0), rotationalSpeed(0)
{
}

AirDriveMessage::AirDriveMessage(double altitudeSpeed, double linearSpeed, double lateralSpeed, double rotationalSpeed):
    altitudeSpeed(altitudeSpeed), linearSpeed(linearSpeed), lateralSpeed(lateralSpeed), rotationalSpeed(rotationalSpeed)
{
}

AirDriveMessage::~AirDriveMessage()
{
}

double AirDriveMessage::getAltitudeSpeed() const
{
    return altitudeSpeed;
}

double AirDriveMessage::getLinearSpeed() const
{
    return linearSpeed;
}

double AirDriveMessage::getLateralSpeed() const
{
    return lateralSpeed;
}

double AirDriveMessage::getRotationalSpeed() const
{
    return rotationalSpeed;
}

void AirDriveMessage::serializeTo(QDataStream &stream) const
{
    stream << altitudeSpeed << linearSpeed << lateralSpeed << rotationalSpeed;
}

void AirDriveMessage::deserializeFrom(QDataStream &stream)
{
    stream >> altitudeSpeed >> linearSpeed >> lateralSpeed >> rotationalSpeed;
}

}
