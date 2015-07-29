#ifndef DESTINATIONGRAPHICITEM_H
#define DESTINATIONGRAPHICITEM_H

#define MM_DEST_MARKER_WIDTH 2
#define MM_DEST_MARKER_HEIGHT 2

#include <QGraphicsItem>

namespace graphics{

class DestinationGraphicItem: public QGraphicsItem
{
public:
    DestinationGraphicItem(uint id);

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget* widget);
    QRectF boundingRect() const;

    uint getRobotID();

    void select();
    void deselect();

private:
    uint robotID;
    bool isSelected;
};

}
#endif // DESTINATIONGRAPHICITEM_H
