#include "customlabelgraphicitem.h"
#include "baseStation/graphicparams.h"
#include <QPainter>
#include <QCursor>

#define BS_CUSTOM_LABEL_WIDTH 8.0
#define BS_CUSTOM_LABEL_HEIGHT 6.0

namespace graphics{

CustomLabelGraphicItem::CustomLabelGraphicItem(QObject *parent, QString txt) :
    QObject(parent), text(txt), focus(false)
{
    this->scale(1.0/SLAM_SCALE_FACTOR, -1.0/SLAM_SCALE_FACTOR);
    this->translate(BS_CUSTOM_LABEL_WIDTH / 2, BS_CUSTOM_LABEL_HEIGHT / 2);
    this->setAcceptHoverEvents(true);
    setCursor(Qt::WhatsThisCursor);
}

void CustomLabelGraphicItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *options, QWidget *widget)
{
    Q_UNUSED(widget); Q_UNUSED(options);
    painter->setBrush(QBrush(Qt::green, Qt::SolidPattern));
    painter->drawRect(0,0, BS_CUSTOM_LABEL_WIDTH, BS_CUSTOM_LABEL_HEIGHT);
    QPen p = painter->pen();
    p.setWidth(1);
    painter->setPen(p);
    painter->drawLine(0,0,BS_CUSTOM_LABEL_WIDTH, BS_CUSTOM_LABEL_HEIGHT);
    painter->drawLine(BS_CUSTOM_LABEL_WIDTH, 0, 0, BS_CUSTOM_LABEL_HEIGHT);
    if(focus)
        painter->drawText(BS_CUSTOM_LABEL_WIDTH*2, BS_CUSTOM_LABEL_HEIGHT*2,text);
}

QRectF CustomLabelGraphicItem::boundingRect() const
{
    return QRectF(0, 0, BS_CUSTOM_LABEL_WIDTH, BS_CUSTOM_LABEL_HEIGHT);
}

void CustomLabelGraphicItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    Q_UNUSED(event);
    this->focus = true;
    update();
}

void CustomLabelGraphicItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    Q_UNUSED(event);
    this->focus = false;
    update();
}

}
