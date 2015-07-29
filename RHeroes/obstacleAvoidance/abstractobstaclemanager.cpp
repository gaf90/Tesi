#include "abstractobstaclemanager.h"



AbstractObstacleManager::AbstractObstacleManager(bool status)
{
    this->status = status;
}

void AbstractObstacleManager::setStatus(bool isActive)
{
    this->status = isActive;
}

bool AbstractObstacleManager:: getStatus()
{
    return status;
}


