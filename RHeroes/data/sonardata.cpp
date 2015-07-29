#include "sonardata.h"

namespace Data{
SonarData::SonarData()
{
}

SonarData::SonarData(double *aFrontSonar, double *aBackSonar,
                     QString aminSonarName, QString aMaxSonarName,
                     double aMinDistance, double aMaxDistance,
                     double aBMinDistance, double aBMaxDistance,
                     SonarPosition aSonarPosition, SLAM::Geometry::LineSegment aSensedObstacle) :
    frontSonar(aFrontSonar),
    backSonar(aBackSonar),
    minSonarName(aminSonarName),
    maxSonarName(aMaxSonarName),
    minFrontDistance(aMinDistance),
    maxFrontDistance(aMaxDistance),
    minBackDistance(aBMinDistance),
    maxBackDistance(aBMaxDistance),
    sonarPosition(aSonarPosition),
    sensedObstacle(aSensedObstacle)
{
}

SonarData::~SonarData()
{
}

double SonarData::getMinDistance() const
{
    return minFrontDistance;
}

double SonarData::getMaxDistance() const
{
    return maxFrontDistance;
}

double SonarData::getMinBackDistance() const
{
    return minBackDistance;
}

double SonarData::getMaxBackDistance() const
{
    return maxBackDistance;
}

double SonarData::getBack(int i) const
{
    return backSonar[i];
}

double SonarData::getFront(int i) const
{
    return frontSonar[i];
}


QString SonarData::getMinSonarName() const
{
    return minSonarName;
}

QString SonarData::getMaxSonarName() const
{
    return maxSonarName;
}

SonarData::SonarPosition SonarData::getPosition() const
{
    return sonarPosition;
}

SLAM::Geometry::LineSegment SonarData::getSensedObstacle() const
{
    return sensedObstacle;
}
}


