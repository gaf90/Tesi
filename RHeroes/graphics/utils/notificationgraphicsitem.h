#ifndef NOTIFICATIONGRAPHICSITEM_H
#define NOTIFICATIONGRAPHICSITEM_H

#include <QGraphicsItem>

#define MM_NOTIFICATION_ITEM_WIDTH 3.
#define MM_NOTIFICATION_ITEM_HEIGHT 4.

namespace graphics{

class NotificationGraphicsItem : public QGraphicsItem
{
public:
    NotificationGraphicsItem();

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* , QWidget* );
    QRectF boundingRect() const;
};

}
#endif // NOTIFICATIONGRAPHICSITEM_H
