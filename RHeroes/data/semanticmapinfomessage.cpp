#include "semanticmapinfomessage.h"

namespace Data{

SemanticMapInfoMessage::SemanticMapInfoMessage():
    area(QList<QPoint>()),
    information("none"),
    center(QPoint())
{
}

SemanticMapInfoMessage::SemanticMapInfoMessage(QList<QPoint> area, QString information):
    area(area),
    information(information),
    center(QPoint(-1,-1))
{

}

SemanticMapInfoMessage::SemanticMapInfoMessage(QPoint center, QString information):
    area(QList<QPoint>()),
    information(information),
    center(center)
{

}

SemanticMapInfoMessage::~SemanticMapInfoMessage()
{
}

QList<QPoint> SemanticMapInfoMessage::getArea() const
{
    return area;
}

QString SemanticMapInfoMessage::getSemanticInfo() const
{
    return information;
}

QPoint SemanticMapInfoMessage::getCenter() const
{
    return center;
}

void SemanticMapInfoMessage::serializeTo(QDataStream &stream) const
{
    stream << center << area << information;
}

void SemanticMapInfoMessage::deserializeFrom(QDataStream &stream)
{
    stream >> center >> area >> information;
}

}
