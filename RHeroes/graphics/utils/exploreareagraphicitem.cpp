#include "exploreareagraphicitem.h"

#include <QPainter>

namespace graphics{

ExploreAreaGraphicItem::ExploreAreaGraphicItem()
{
}

void ExploreAreaGraphicItem::paint(QPainter * painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->drawEllipse(QPoint(0, 0), 2, 2);
    painter->drawEllipse(QPoint(0, 0), 4, 4);
    painter->drawEllipse(QPoint(0, 0), 6, 6);
}

QRectF ExploreAreaGraphicItem::boundingRect() const
{
    return QRectF(0,0,6,6);
}

void ExploreAreaGraphicItem::select()
{
    this->show();
}

void ExploreAreaGraphicItem::deselect()
{
    this->hide();
}

}
