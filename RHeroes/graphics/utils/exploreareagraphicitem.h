#ifndef EXPLOREAREAGRAPHICITEM_H
#define EXPLOREAREAGRAPHICITEM_H

#include "graphics/utils/waypointmarkergraphicsitem.h"

namespace graphics{

class ExploreAreaGraphicItem: public WaypointMarkerGraphicsItem
{
public:
    ExploreAreaGraphicItem();

    void paint(QPainter *, const QStyleOptionGraphicsItem *, QWidget *);
    QRectF boundingRect() const;

    void select();
    void deselect();
};

}
#endif // EXPLOREAREAGRAPHICITEM_H
