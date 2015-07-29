#ifndef SENSORDATA_H
#define SENSORDATA_H

/*!
  The Message class represents a common interface for sensorial data
  notification used by Sensor classes

  \see Sensor
 */
class SensorData
{
public:
    /*!
      Dummy virtual destructor needed to enable RTTI support
     */
    virtual ~SensorData() {}
};

#endif // SENSORDATA_H
