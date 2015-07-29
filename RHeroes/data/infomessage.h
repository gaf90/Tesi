#ifndef INFOMESSAGE_H
#define INFOMESSAGE_H

#include "data/message.h"
#include "data/serializable.h"

namespace Data{

/**
 * @brief generic message for information transmission to/from the base station
 *
 * The InfoMessage class is a container for the transmission of a message
 * containing generic information. It can be used to send or request information about
 * connectivity, battery status, pitch and roll angles, fault detections and request for
 * supervisor intervention. It can also be used to set some parameters, like framerate...
 *
 * @see
 */
class InfoMessage : public Message, public Serializable
{
public:
    InfoMessage();

    /**
    * Destroys the InfoMessage object.
    */
    virtual ~InfoMessage();

    /**
    * Creates an InfoMessage with the information requested in the QString info.
    * @param infoName the name of the information requested.
    * @param info the information content of the message. It must be properly structured
    */
    InfoMessage(QString infoName, QString info);

    /**
    * Creates an InfoMessage to request an information
    * @param infoName the name of the information requested
    */
    InfoMessage(QString infoName);

    /**
    * @return the infoName of the requested/arrived information
    */
    QString getInfoName() const;

    /**
    * @return the info arrived as a QString
    */
    QString getInfo() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);


private:

    QString infoName;
    QString info;
};

}

#endif // INFOMESSAGE_H
