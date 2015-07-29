#ifndef MAPMESSAGE_H
#define MAPMESSAGE_H

#include "serializable.h"
#include "message.h"
#include "slam/map.h"

namespace Data{

/**
 * @brief Map message to Base Station
 *
 *
 * Sends a map object to the base station.
 *
 * @see
 */
class MapMessage: public Serializable, Message
{
public:
    MapMessage();

    /**
    * Builds a MapMessage object with the relative Map.
    *
    * @param mappa l'oggetto Map da mandare alla base station
    */
    MapMessage(SLAM::Map map, bool estimated);

    /**
    * Destroys the mapMessage object
    */
    virtual ~MapMessage();

    /**
    * return the map object
    */
    SLAM::Map getMap() const;

    /**
    * return true if the Map object in the container has been estimated by
    * the semantic information extraction module.
    */
    bool isEstimated() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);



private:


    SLAM::Map map;
    bool estimated;

};
}

#endif // MAPMESSAGE_H
