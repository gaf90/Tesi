#include "victim.h"
#include "shared/constants.h"
#include "shared/utilities.h"
#define UNKNOWN_ROBOT 999
#define UNKNOWN_TIME 0

namespace BaseStation{

Victim::Victim(uint discoveredBy, uint id, Data::Pose *p):
    victimID(id), discoveringRobot(discoveredBy), assignedRobot(UNKNOWN_ROBOT),
    position(new Data::Pose(*p)), images(new QList<QImage>()), timeToReach(UNKNOWN_TIME),
    confirmed(false)
{
}

Victim::Victim(uint id, Data::Pose *p):
    victimID(id), discoveringRobot(BASE_STATION_ID), assignedRobot(UNKNOWN_ROBOT),
    position(new Data::Pose(*p)), images(new QList<QImage>()), timeToReach(UNKNOWN_TIME),
    confirmed(false)
{
}

Victim::~Victim()
{
//    delete position;
}

void Victim::assignToRobot(uint robot)
{
    this->assignedRobot = robot;
}

void Victim::setTimeToReach(int time)
{
    this->timeToReach = time;
}

void Victim::addImage(const QImage &img)
{
    images->append(img);
}

bool Victim::equals(BaseStation::Victim v)
{
    //TO BE REFINED: now the victims are the same if they have the same id or almost the same position
    if(v.victimID == this->victimID)
        return true;
    else if(v.position->getDistance(*this->position) < VM_EQUALITY_DISTANCE_TH &&
            abs(this->getPosition().getTheta() - v.getPosition().getTheta()) < VM_EQUALITY_ANGLE_TH)
        return true;
    else return false;
}

bool Victim::isConfirmed()
{
    return confirmed;
}

void Victim::confirm()
{
    this->confirmed = true;
}

void Victim::setPosition(const Data::Pose &p)
{
    this->position = new Data::Pose(p);
}

uint Victim::getVictimID()
{
    return this->victimID;
}

uint Victim::getDiscoverererID()
{
    return this->discoveringRobot;
}

uint Victim::getAssignedRobotID()
{
    return this->assignedRobot;
}

QString Victim::printAssignedRobotName()
{
    if(assignedRobot == UNKNOWN_ROBOT)
        return "none";
    else
        return robotNameFromIndex(assignedRobot);
}

const Data::Pose & Victim::getPosition() const
{
    return *position;
}

const QList<QImage> &Victim::getImages() const
{
    return *this->images;
}

const QImage &Victim::getImage(int i) const
{
    return this->images->at(i);
}

int Victim::getTimeToBeReached()
{
    return timeToReach;
}

QString Victim::printTimeToBeReached()
{
    if(timeToReach == UNKNOWN_TIME)
        return "unknown";
    else
        return QString::number(timeToReach);
}

}

