#ifndef VICTIMGRAPHICITEM_H
#define VICTIMGRAPHICITEM_H

#include <QObject>
#include <QGraphicsItem>
#include "data/pose.h"

namespace graphics{

class VictimGraphicItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit VictimGraphicItem(QObject *parent = 0, uint id = 0, QWidget *parentWidget = 0);

    uint getVictimID();

    void select();
    void deselect();

    void setPos(const QPointF &pos);
    void setPos(qreal x, qreal y);

    /*Overrided from QgraphicsItem*/
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget* widget);
    QRectF boundingRect() const;

    void setParentWidget(QWidget* parentWidget);

signals:
    void signalSelectedVictim(uint id);
    void signalVictimMoved(uint id, const Data::Pose &position);

public slots:

private:

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);


    uint victimID;
    bool isSelected;
    QPointF position;
    QWidget *parentWidget;
};

}
#endif // VICTIMGRAPHICITEM_H
