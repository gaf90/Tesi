#ifndef WAYPOINTMARKERGRAPHICSITEM_H
#define WAYPOINTMARKERGRAPHICSITEM_H

#include <QGraphicsItem>

namespace graphics{

class WaypointMarkerGraphicsItem: public QGraphicsItem
{
public:
    WaypointMarkerGraphicsItem();

    /*Overrided from QgraphicsItem*/
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*);
    QRectF boundingRect() const;

    void select();
    void deselect();

private:
    bool selected;
};

}
#endif // WAYPOINTMARKERGRAPHICSITEM_H
