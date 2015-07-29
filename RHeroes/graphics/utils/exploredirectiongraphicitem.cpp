#include "exploredirectiongraphicitem.h"
#include <QPainter>
#include "baseStation/graphicparams.h"
#include "shared/utilities.h"
#include <cmath>

namespace graphics{

ExploreDirectionGraphicItem::ExploreDirectionGraphicItem(double xVal, double yVal):
    x(xVal), y(yVal)
{
    this->translate(ROBOT_MARKER_WIDTH * sgn(x) * 0.5, ROBOT_MARKER_HEIGHT * sgn(y) * 0.5);
    bounds = QRectF(QPoint(-x, y),QPoint(x, -y));
}

void ExploreDirectionGraphicItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    QBrush b = QBrush(Qt::black, Qt::SolidPattern);
    painter->setBrush(b);
//    QPen p = painter->pen();
//    p.setWidth();
//    painter->setPen(p);
    if((!checkDouble(x))||(!checkDouble(y))){
        return;
    }
    painter->drawLine(0,0,x,y);
    ldbg << "x,y = " << x << "," << y << endl;
    double s = sin(atan(y/x));
    double c = cos(atan(y/x));
    if(x < 0)
    {
        s = -s;
        c = -c;
    }
    ldbg << "sin: " << s << " , cos: " << c << endl;
    double arrowX = 2, arrowY = 1.5;
    QPointF points[3]={
        QPointF(x,y),
        QPointF(x - (arrowX*c - arrowY*s), y - (arrowX*s + arrowY*c)),
        QPointF(x - (arrowX*c + arrowY*s), y - (arrowX*s - arrowY*c))
    };
    painter->drawConvexPolygon(points, 3);
}

QRectF ExploreDirectionGraphicItem::boundingRect() const
{
    return bounds;
}

void ExploreDirectionGraphicItem::updateVector(double xVal, double yVal)
{
    x = xVal;
    y = yVal;
    prepareGeometryChange();
    bounds = QRectF(QPoint(0, 0),QPoint(2*x, 2*y));
    update();
}

double ExploreDirectionGraphicItem::getX()
{
    return x;
}

double ExploreDirectionGraphicItem::getY()
{
    return y;
}

bool ExploreDirectionGraphicItem::checkDouble(double d){
    if (d != d){
        return false;
    } else if (d > std::numeric_limits<qreal>::max()){
        return false;//return "+Inf";
    } else if (d < -std::numeric_limits<qreal>::max()){
        return false;
    }
    return true;
}

}


