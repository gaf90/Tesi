/*
 * mapviewwidget.h
 *
 *  Created on: 28/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef MAPVIEWWIDGET_H_
#define MAPVIEWWIDGET_H_

#include "slam/geometry/linesegment.h"
#include "slam/geometry/pointscan.h"
#include "slam/map.h"
#include <QGraphicsView>

namespace SLAM {
namespace Dataset {

class MapViewWidget : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MapViewWidget(QWidget *parent = 0);
    virtual ~MapViewWidget();

    void drawReferenceFrame();
    void setKeepRatio(bool kr);
    virtual int heightForWidth(int w) const;
    void reset();
    double scaleValue() const;

signals:
    void zoomChanged(double zoom);

public slots:
    void setMap(const Map &map);
    void setScan(const Geometry::PointScan &scan);

protected slots:
    virtual void wheelEvent(QWheelEvent *event);
    virtual void resizeEvent(QResizeEvent *event);

private:
    void drawNode(const PathNode &node, double size,
                  const QPen &pen, const QBrush &fill,
                  bool withcovariance = false, int zvalue = 3);

private:
    QGraphicsScene *qscene;
    double zoom, zoomMultiplier;
    double minZoom, maxZoom;
    QPen linePen, pointPen, framePen, noPen;
    QBrush pointBrush, noBrush;
    bool keepRatio, scanMode;
};

} /* namespace Dataset */
} /* namespace SLAM */

#endif /* MAPVIEWWIDGET_H_ */
