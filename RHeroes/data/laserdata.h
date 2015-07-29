#ifndef LASERDATA_H
#define LASERDATA_H

#include "message.h"
#include <QList>

namespace Data {

/**
 * @brief Laser data message
 *
 * This class represents laser data readings, signalled by LaserSensor
 *
 * @see LaserSensor
 */
class LaserData : public Message
{
public:
    /**
     * Constructs a new LaserData message, containing the readings and
     * properties of the laser sensor
     *
     * @param timestamp Timestamp of when the scan was taken
     * @param fov The field of view of the laser sensor
     * @param resolution The tick resolution of the laser sensor (in radiants)
     * @param readings The list of dinstance readings
     */
    LaserData(double timestamp, double fov, double resolution, const QList<double> &readings);

    /**
     * Destroys the LaserData
     */
    virtual ~LaserData();

    /**
     * @return The timestamp of when the scan was taken
     */
    double getTimestamp() const;

    /**
     * @return A const reference to the list of distance readings
     */
    const QList<double> &getReadings() const;

    /**
     * @return The field of view of the laser sensor
     */
    double getFOV() const;

    /**
     * @return The tick resolution of the laser sensor (in radiants)
     */
    double getResolution() const;

private:
    double timestamp, fov, resolution;
    QList<double> readings;

};

}

#endif // LASERDATA_H
