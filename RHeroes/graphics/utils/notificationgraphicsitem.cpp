#include "notificationgraphicsitem.h"
#include <QPainter>

namespace graphics{

NotificationGraphicsItem::NotificationGraphicsItem()
{
    scale(-1,-1);
    translate(-MM_NOTIFICATION_ITEM_WIDTH/2, -MM_NOTIFICATION_ITEM_HEIGHT/2);
}

void NotificationGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
    static const QPointF points[3]={
        QPointF(0,0),
        QPointF(MM_NOTIFICATION_ITEM_WIDTH, 0),
        QPointF(MM_NOTIFICATION_ITEM_WIDTH/2, MM_NOTIFICATION_ITEM_HEIGHT)
    };
    painter->drawConvexPolygon(points, 3);
    painter->drawEllipse(MM_NOTIFICATION_ITEM_WIDTH/2, MM_NOTIFICATION_ITEM_HEIGHT+1, 1, 1);
}

QRectF NotificationGraphicsItem::boundingRect() const
{
    return QRectF(0,0, MM_NOTIFICATION_ITEM_WIDTH, MM_NOTIFICATION_ITEM_HEIGHT);
}

}
