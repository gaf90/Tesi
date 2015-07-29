#ifndef EXPLOREDIRECTIONGRAPHICITEM_H
#define EXPLOREDIRECTIONGRAPHICITEM_H

#include <QGraphicsItem>

#define BS_MM_DIRECTION_THRESHOLD 5

namespace graphics{

class ExploreDirectionGraphicItem: public QGraphicsItem
{
public:
    ExploreDirectionGraphicItem(double xVal, double yVal);

    /*Overrided from QgraphicsItem*/
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* , QWidget* );
    QRectF boundingRect() const;

    void updateVector(double xVal, double yVal);
    double getX();
    double getY();

private:
    double x, y;
    QRectF bounds;

    bool checkDouble(double d);

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
};

}
#endif // EXPLOREDIRECTIONGRAPHICITEM_H
