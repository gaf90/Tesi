#ifndef ROBOTGRAPHICITEM_H
#define ROBOTGRAPHICITEM_H

#include <QGraphicsItem>
#include <QObject>
#include "data/pose.h"
#include "baseStation/graphicparams.h"

namespace graphics{

class RobotGraphicItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit RobotGraphicItem(uint id, bool selected);

    virtual ~RobotGraphicItem();

    uint getRobotID() const;

    QString getRobotName() const;

    void select();
    void deselect();

    void setReachable(bool reachable);

    /*Overrided from QgraphicsItem*/
    void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*);
    QRectF boundingRect() const;

signals:
    void sigSelectedRobotChangedRGI(uint id);

public slots:

private:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);

    uint robotID;
    QString robotName;
    bool isSelected;
    bool isReachable;
};

}
#endif // ROBOTGRAPHICITEM_H
