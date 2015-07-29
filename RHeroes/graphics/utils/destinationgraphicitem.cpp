#include "destinationgraphicitem.h"
#include <QPainter>

namespace graphics{

DestinationGraphicItem::DestinationGraphicItem(uint id):
    robotID(id), isSelected(false)
{
}

void DestinationGraphicItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *options, QWidget *widget)
{
    Q_UNUSED(options);    Q_UNUSED(widget);
    QPen p = painter->pen();
    p.setWidth(1);
    if(isSelected)
        p.setColor(Qt::red);
    else
        p.setColor(Qt::blue);
    painter->setPen(p);
    painter->drawLine(0,0,MM_DEST_MARKER_WIDTH, MM_DEST_MARKER_HEIGHT);
    painter->drawLine(MM_DEST_MARKER_WIDTH, 0, 0, MM_DEST_MARKER_HEIGHT);
}

QRectF DestinationGraphicItem::boundingRect() const
{
    return QRectF(0,0,MM_DEST_MARKER_WIDTH,MM_DEST_MARKER_HEIGHT);
}

uint DestinationGraphicItem::getRobotID()
{
    return this->robotID;
}

void DestinationGraphicItem::select()
{
    this->isSelected = true;
    update();
}

void DestinationGraphicItem::deselect()
{
    this->isSelected = false;
    update();
}

}
