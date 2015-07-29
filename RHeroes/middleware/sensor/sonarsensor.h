#ifndef SONARSENSOR_H
#define SONARSENSOR_H

#include <QObject>
#include "sensor.h"
#include "slam/geometry/linesegment.h"

#define SONAR_PER_SIDE 5

namespace Middleware{
class SonarSensor : public Sensor
{
    Q_OBJECT

public:
    /**
     * Constructs a new SonarSensor
     *
     * @param parent Optional QObject parent
     */
    explicit SonarSensor(QObject *parent = 0);

    /**
     * Destroys the SonarSensor
     */
    virtual ~SonarSensor();
    SLAM::Geometry::LineSegment getSensedObstacle() const;

public slots:
    /**
     * Sensor::onMessageReceived() implementation for SonarSensor
     */
    virtual void onMessageReceived(const Data::Message &message);

private:
    double getMinDistance(double sonarArray[]) const;
    double getMaxDistance(double sonarArray[]) const;

    double frontSonar[SONAR_PER_SIDE];
    double backSonar[SONAR_PER_SIDE];
    double frontSonarPosition[SONAR_PER_SIDE];
};
}

#endif // SONARSENSOR_H
