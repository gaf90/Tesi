#include "dangergraphicsitem.h"
#include <QPainter>

#define BS_DANGER_LABEL_WIDTH 7.0
#define BS_DANGER_LABEL_HEIGHT 6.0

#define BS_EXCLAMATION_MARK_WIDTH 2.
#define BS_EXCLAMATION_MARK_HEIGHT 3.

namespace graphics{

DangerGraphicsItem::DangerGraphicsItem()
{
    scale(-1,-1);
}

void DangerGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    static const QPointF points[3]={
        QPointF(BS_DANGER_LABEL_WIDTH,0),
        QPointF(BS_DANGER_LABEL_WIDTH/2,BS_DANGER_LABEL_HEIGHT),
        QPointF(0,0)
    };

    QBrush b = QBrush(Qt::red, Qt::SolidPattern);
    painter->setBrush(b);
    painter->drawConvexPolygon(points, 3);

    painter->setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
    static const QPointF points2[3]={
        QPointF((BS_DANGER_LABEL_WIDTH / 2) - (BS_EXCLAMATION_MARK_WIDTH / 2), 1),
        QPointF(BS_EXCLAMATION_MARK_WIDTH + (BS_DANGER_LABEL_WIDTH / 2) - (BS_EXCLAMATION_MARK_WIDTH / 2), 1),
        QPointF(BS_EXCLAMATION_MARK_WIDTH/2 + (BS_DANGER_LABEL_WIDTH / 2) - (BS_EXCLAMATION_MARK_WIDTH / 2),
                BS_EXCLAMATION_MARK_HEIGHT + 1)
    };
    painter->drawConvexPolygon(points2, 3);
    painter->drawEllipse((BS_DANGER_LABEL_WIDTH / 2) - (BS_EXCLAMATION_MARK_WIDTH / 2) + (BS_EXCLAMATION_MARK_WIDTH / 2),
                          BS_EXCLAMATION_MARK_HEIGHT + 1 , 1, 1);
}

QRectF DangerGraphicsItem::boundingRect() const
{
    return QRectF(0, 0, BS_DANGER_LABEL_WIDTH, BS_DANGER_LABEL_HEIGHT);
}

}
