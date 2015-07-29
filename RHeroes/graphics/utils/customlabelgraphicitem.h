#ifndef CUSTOMLABELGRAPHICITEM_H
#define CUSTOMLABELGRAPHICITEM_H

#include <QObject>
#include <QGraphicsItem>

namespace graphics{

class CustomLabelGraphicItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit CustomLabelGraphicItem(QObject *parent = 0, QString txt = "");

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget* widget);
    QRectF boundingRect() const;
    
signals:
    
public slots:

private:
    QString text;
    bool focus;

    void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);

};

}
#endif // CUSTOMLABELGRAPHICITEM_H
