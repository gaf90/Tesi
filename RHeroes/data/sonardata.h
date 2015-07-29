#ifndef SONARDATA_H
#define SONARDATA_H

#include "message.h"
#include "slam/geometry/linesegment.h"

namespace Data{
class SonarData : public Message
{
public:
    enum SonarPosition{
        Back,
        Front
    };

    SonarData();
    SonarData(double *frontSonar, double *backSonar,
              QString aMinSonarName, QString aMaxSonarName,
              double minFrontDistance, double maxDistance,
              double minBDistance, double maxBDistance,
              SonarPosition sonarPosition, SLAM::Geometry::LineSegment sensedObstacle);
    virtual ~SonarData();

    double getMinDistance() const;
    double getMaxDistance() const;
    double getMinBackDistance() const;
    double getMaxBackDistance() const;
    double getBack(int i) const;
    double getFront(int i) const;
    QString getMinSonarName() const;
    QString getMaxSonarName() const;

    SonarPosition getPosition() const;
    SLAM::Geometry::LineSegment getSensedObstacle() const;

private:
    double minFrontDistance;
    double maxFrontDistance;
    double minBackDistance;
    double maxBackDistance;
    SonarPosition sonarPosition;
    SLAM::Geometry::LineSegment sensedObstacle;
    QString minSonarName;
    QString maxSonarName;
    double *frontSonar;
    double *backSonar;
};
}
#endif // SONARDATA_H
