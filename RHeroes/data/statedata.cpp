#include "statedata.h"

namespace Data{

StateData::StateData()
{
}

StateData::StateData(const QString &vehicleType, float time, float frontSteer,
          float rearSteer, bool lightToggle, int lightIntensity, int battery) :
    vehicleType(vehicleType), time(time), frontSteer(frontSteer), rearSteer(rearSteer),
    lightToggle(lightToggle), lightIntensity(lightIntensity), battery(battery)
{

}

StateData::~StateData()
{

}

void StateData::setVehicleType(const QString &vehicleType)
{
    this->vehicleType = vehicleType;
}

void StateData::setTime(float time)
{
    this->time = time;
}

void StateData::setFrontSteer(float frontSteer)
{
    this->frontSteer = frontSteer;
}

void StateData::setRearSteer(float rearSteer)
{
    this->rearSteer = rearSteer;
}

void StateData::setLightToggle(bool lightToggle)
{
    this->lightToggle = lightToggle;
}

void StateData::setLightIntensity(int lightIntensity)
{
    this->lightIntensity = lightIntensity;
}

void StateData::setBattery(int battery)
{
    this->battery = battery;
}

QString StateData::getVehicleType() const
{
    return vehicleType;
}

float StateData::getTime() const
{
    return time;
}

float StateData::getFrontSteer() const
{
    return frontSteer;
}

float StateData::getRearSteer() const
{
    return rearSteer;
}

bool StateData::getLightToggle() const
{
    return lightToggle;
}

int StateData::getLightIntensity() const
{
    return lightIntensity;
}

int StateData::getBattery() const
{
    return battery;
}
}
