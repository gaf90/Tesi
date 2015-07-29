#ifndef SEMANTICMAPINFOMESSAGE_H
#define SEMANTICMAPINFOMESSAGE_H

#include "message.h"
#include "serializable.h"
#include <QPoint>

namespace Data{

class SemanticMapInfoMessage: public Message, public Serializable
{

public:
    SemanticMapInfoMessage();

    /**
    * Creates a SemanticMapInfoMessage with a string, the semantic information,
    * and a list of points characterizing the area interested.
    *
    * @param area the list of points that identify the perimeter of the considered area
    * @param information the semantic info associated to the point/area
    */
    SemanticMapInfoMessage(QList<QPoint> area, QString information);


    /**
    * Creates a SemanticMapInfoMessage with the center point of the area considered
    * and the relative semantic information into a QString
    *
    * @param center the center of the area considered
    * @param information the semantic info associated to the point/area
    */
    SemanticMapInfoMessage(QPoint center, QString information);

    virtual ~SemanticMapInfoMessage();

    /**
    * @return the list of points composing the perimeter of the considered area
    */
    QList<QPoint> getArea() const;

    /**
    * @return the QString describing the semantic information transferred
    */
    QString getSemanticInfo() const;

    /**
    * @return the QPoint, center of the considered areas
    */
    QPoint getCenter() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);


private:
    QList<QPoint> area;
    QString information;
    QPoint center;

};

}


#endif // SEMANTICMAPINFOMESSAGE_H
