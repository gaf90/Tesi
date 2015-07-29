#include "waypointmarkergraphicsitem.h"
#include <QPainter>

namespace graphics{

WaypointMarkerGraphicsItem::WaypointMarkerGraphicsItem()
{
    selected = true;
}

void WaypointMarkerGraphicsItem::paint(QPainter * painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    if(selected){
        painter->setBrush(QBrush(QColor(Qt::red)));
        painter->drawEllipse(QPointF(0,0),2,2);
        painter->setBrush(QBrush(QColor(Qt::blue)));
        painter->drawEllipse(QPointF(0,0),1,1);
        painter->setBrush(QBrush(QColor(Qt::red)));
        painter->drawEllipse(QPointF(0,0),0.5,0.5);
    }
    else{
        painter->setBrush(QBrush(QColor(Qt::blue)));
        painter->drawEllipse(QPointF(0,0),2,2);
        painter->setBrush(QBrush(QColor(Qt::red)));
        painter->drawEllipse(QPointF(0,0),1,1);
        painter->setBrush(QBrush(QColor(Qt::blue)));
        painter->drawEllipse(QPointF(0,0),0.5,0.5);
    }
}

QRectF WaypointMarkerGraphicsItem::boundingRect() const
{
    return QRectF(-2,-2,2,2);
}

void WaypointMarkerGraphicsItem::deselect()
{
    selected = false;
}

void WaypointMarkerGraphicsItem::select()
{
    selected = true;
}

}
