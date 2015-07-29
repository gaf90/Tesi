#include "victimgraphicitem.h"
#include "baseStation/graphicparams.h"

#include <QPainter>
#include <QGraphicsSceneDragDropEvent>
#include <QDrag>
#include <QCursor>
#include <QApplication>
#include <QMimeData>
#include <QMessageBox>

namespace graphics{

VictimGraphicItem::VictimGraphicItem(QObject *parent, uint id, QWidget * parentWidget) :
    QObject(parent), victimID(id), isSelected(false), parentWidget(parentWidget)
{
    this->setAcceptDrops(true);
    setCursor(Qt::OpenHandCursor);

    this->scale(-1,-1);
    this->translate(-1,-2.5);
}

uint VictimGraphicItem::getVictimID()
{
    return this->victimID;
}

void VictimGraphicItem::select()
{
    this->isSelected = true;
}

void VictimGraphicItem::deselect()
{
    this->isSelected = false;
}
void VictimGraphicItem::setPos(const QPointF &pos)
{
    QGraphicsItem::setPos(pos);
    this->position = pos;
}

void VictimGraphicItem::setPos(qreal x, qreal y)
{
    QGraphicsItem::setPos(x, y);
    this->position = QPointF(x,y);
}

void VictimGraphicItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    Q_UNUSED(event);
    setCursor(QCursor(Qt::ClosedHandCursor));
    this->select();
    emit signalSelectedVictim(victimID);
}

void VictimGraphicItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    Q_UNUSED(event);
    setCursor(Qt::OpenHandCursor);
}

void VictimGraphicItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (QLineF(event->screenPos(),
               event->buttonDownScreenPos(Qt::LeftButton)).length() < QApplication::startDragDistance())
    {
            return;
    }
    else{
        QDrag *drag;
        if(parentWidget == 0)
            drag = new QDrag(event->widget());
        else
            drag = new QDrag(parentWidget);
        QMimeData *mime = new QMimeData;
        QByteArray itemData;
        QDataStream dataStream(&itemData, QIODevice::WriteOnly);
        dataStream << victimID;
        mime->setData("application/x-dnditemdata", itemData);
        drag->setMimeData(mime);
        drag->exec();
    }
}

QRectF VictimGraphicItem::boundingRect() const
{
    qreal penWidth = 1;
    return QRectF(0, 0,
                  VICTIM_MARKER_WIDTH + penWidth, VICTIM_MARKER_HEIGHT + penWidth);
}

void VictimGraphicItem::setParentWidget(QWidget *parentWidget)
{
    this->parentWidget = parentWidget;
}

void VictimGraphicItem::paint(QPainter * painter, const QStyleOptionGraphicsItem * options, QWidget * widget)
{
    Q_UNUSED(widget); Q_UNUSED(options);
    QBrush b;
    if(isSelected)
        b = QBrush(Qt::red, Qt::SolidPattern);
    else
        b = QBrush(Qt::blue, Qt::SolidPattern);
    painter->setBrush(b);
    //Drawing a stupid man
    painter->drawEllipse(QPoint(1,1),1,1);
    painter->drawLine(QPoint(1,2),QPoint(1,4));
    painter->drawLine(QPoint(0,3),QPoint(2,3));
    painter->drawLine(QPoint(1,4),QPoint(0,5));
    painter->drawLine(QPoint(1,4),QPoint(2,5));
}

}
