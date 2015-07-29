#include "robotgraphicitem.h"
#include "shared/utilities.h"
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

namespace graphics{
RobotGraphicItem::RobotGraphicItem(uint id, bool selected):
    robotID(id),
    robotName(robotNameFromIndex(robotID)),
    isSelected(selected), isReachable(true)
{
    this->setAcceptHoverEvents(true);
    this->translate(-ROBOT_MARKER_WIDTH/2 , -ROBOT_MARKER_HEIGHT/2);
    this->setTransformOriginPoint(ROBOT_MARKER_WIDTH/2 , ROBOT_MARKER_HEIGHT/2);
}

RobotGraphicItem::~RobotGraphicItem()
{
}

void RobotGraphicItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);   Q_UNUSED(option);
    static const QPointF points[3]={
        QPointF(ROBOT_MARKER_WIDTH,0),
        QPointF(ROBOT_MARKER_WIDTH/2,ROBOT_MARKER_HEIGHT),
        QPointF(0,0)
    };
    QBrush b;
    if(isSelected)
        b = QBrush(Qt::red, Qt::SolidPattern);
    else
    {
        if(isReachable)
            b = QBrush(Qt::blue, Qt::SolidPattern);
        else
            b = QBrush(Qt::gray, Qt::SolidPattern);
    }
    painter->setBrush(b);
    painter->drawConvexPolygon(points, 3);
     painter->drawText(QPointF(ROBOT_MARKER_WIDTH,ROBOT_MARKER_HEIGHT),robotName.mid(6));
}

QRectF RobotGraphicItem::boundingRect() const
{
    qreal penWidth = 0;
    return QRectF(0, 0,
                    ROBOT_MARKER_WIDTH + penWidth, ROBOT_MARKER_HEIGHT + penWidth);
}

uint RobotGraphicItem::getRobotID() const
{
    return this->robotID;
}

QString RobotGraphicItem::getRobotName() const
{
    return this->robotName;
}

void RobotGraphicItem::select()
{
    this->isSelected = true;
    update();
}

void RobotGraphicItem::deselect()
{
    this->isSelected = false;
    update();
}

void RobotGraphicItem::setReachable(bool reachable)
{
    this->isReachable = reachable;
}

void RobotGraphicItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton)
    {
        //Check sul fatto che il puntatore sia sull'oggetto???
        this->select();
        emit sigSelectedRobotChangedRGI(this->robotID);
        event->accept();
    }
    else
        event->ignore();
}

}


