#ifndef DANGERGRAPHICSITEM_H
#define DANGERGRAPHICSITEM_H

#include <QGraphicsItem>

namespace graphics{

class DangerGraphicsItem : public QGraphicsItem
{
public:
    DangerGraphicsItem();

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* , QWidget* );
    QRectF boundingRect() const;


};

}
#endif // DANGERGRAPHICSITEM_H
