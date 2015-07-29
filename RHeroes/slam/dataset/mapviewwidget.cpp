/*
 * mapviewwidget.cpp
 *
 *  Created on: 28/nov/2012
 *      Author: Mladen Mazuran
 */

#include "mapviewwidget.h"
#include "datasetconstants.h"
#include "slam/utilities.h"
#include "slam/geometry/segmentscan.h"
#include "slam/geometry/uncertainlinesegment.h"
#include "shared/config.h"
#include <QWheelEvent>
#include <QGraphicsEllipseItem>
#include <QtAlgorithms>
#if defined(DS_ENABLE_OPENGL) && DS_ENABLE_OPENGL
#   include <QGLWidget>
#endif
#include <Eigen/Cholesky>

namespace SLAM {
namespace Dataset {

MapViewWidget::MapViewWidget(QWidget *parent) :
    QGraphicsView(parent), qscene(new QGraphicsScene(this)),
    zoom(10), zoomMultiplier(1.1220184543), minZoom(1), maxZoom(100),
    linePen(Qt::black, 1.2, Qt::SolidLine, Qt::RoundCap),
    pointPen(Qt::red, 0.5, Qt::SolidLine, Qt::RoundCap),
    framePen(Qt::gray, 0.5, Qt::SolidLine), noPen(Qt::white),
    pointBrush(Qt::red), noBrush(Qt::NoBrush), keepRatio(false), scanMode(false)
{
    setScene(qscene);
    scale(zoom, zoom);
    linePen.setCosmetic(true);
    pointPen.setCosmetic(true);
    framePen.setCosmetic(true);

    QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);
    setBackgroundBrush(QBrush(QColor(216, 205, 237), Qt::SolidPattern));
#if defined(DS_ENABLE_OPENGL) && DS_ENABLE_OPENGL
    /*
        Looks like enabling OpenGL doesn't really improve performance, or I'm doing it wrong.
        Either way disable it, as it gives no upper hands.
    */
    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DirectRendering)));
    setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
#endif
}

MapViewWidget::~MapViewWidget()
{
    delete qscene;
}

void MapViewWidget::drawReferenceFrame()
{
    qscene->addEllipse(-DS_FRAME_RANGE, -DS_FRAME_RANGE,
                      2 * DS_FRAME_RANGE, 2 * DS_FRAME_RANGE,
                      framePen, noBrush)->setZValue(1);
    qscene->addEllipse(-5 * DS_FRAME_RANGE, -5 * DS_FRAME_RANGE,
                      10 * DS_FRAME_RANGE, 10 * DS_FRAME_RANGE,
                      /*noPen*/QPen(this->backgroundBrush().color()), noBrush)->setZValue(1);
    qscene->addLine(-DS_FRAME_RANGE - DS_FRAME_TICK_SIZE, 0,
                    DS_FRAME_RANGE + DS_FRAME_TICK_SIZE, 0, framePen)->setZValue(1);
    qscene->addLine(0, -DS_FRAME_RANGE - DS_FRAME_TICK_SIZE, 0,
                    DS_FRAME_RANGE + DS_FRAME_TICK_SIZE, framePen)->setZValue(1);
    for(int i = - (int) DS_FRAME_RANGE + 1; i < (int) DS_FRAME_RANGE; i++) {
        if(i == 0) continue;
        qscene->addLine(i, -DS_FRAME_TICK_SIZE/2, i, DS_FRAME_TICK_SIZE/2, framePen)->setZValue(1);
        qscene->addLine(-DS_FRAME_TICK_SIZE/2, i, DS_FRAME_TICK_SIZE/2, i, framePen)->setZValue(1);
    }
    double theta = DS_FRAME_ANGLE_TICK_SIZE;
    while(theta < 2 * M_PI) {
        const double c = std::cos(theta), s = std::sin(theta);
        qscene->addLine(
                     c * (DS_FRAME_RANGE + DS_FRAME_TICK_SIZE / 2),
                    -s * (DS_FRAME_RANGE + DS_FRAME_TICK_SIZE / 2),
                     c * (DS_FRAME_RANGE - DS_FRAME_TICK_SIZE / 2),
                    -s * (DS_FRAME_RANGE - DS_FRAME_TICK_SIZE / 2), framePen)->setZValue(1);
        theta += DS_FRAME_ANGLE_TICK_SIZE;
    }

}

void MapViewWidget::drawNode(const PathNode &node, double size, const QPen &pen,
        const QBrush &fill, bool withcovariance, int zvalue)
{
    QPolygonF poly;
    QPointF base(node.x(), -node.y());
    QPointF head(node.x() + size * std::cos(node.theta() + M_PI_2),
            -node.y() - size * std::sin(node.theta() + M_PI_2));
    QPointF delta = head - base;
    poly.append(head);
    poly.append(base + QPointF(-delta.y(), delta.x()) / std::sqrt(12));
    poly.append(base - QPointF(-delta.y(), delta.x()) / std::sqrt(12));
    qscene->addPolygon(poly, pen, fill)->setZValue(zvalue);

    if(withcovariance && node.hasCovariance()) {
        Eigen::Matrix2d cov = node.covariance().block<2, 2>(0, 0);
        cov(0,1) = cov(1,0) *= -1; // Qt reference frame has y mirrored
        Eigen::Matrix2d covChol = cov.llt().matrixL();

        QGraphicsEllipseItem *ell = new QGraphicsEllipseItem(-3, -3, 6, 6);
        ell->setPen(pen);
        ell->setBrush(noBrush);
        ell->setTransform(QTransform(
                covChol(0,0), covChol(0,1),
                covChol(1,0), covChol(1,1),
                node.x(), -node.y()));
        ell->setZValue(zvalue);
        qscene->addItem(ell);
    }
}

void MapViewWidget::setMap(const Map &map)
{
    scanMode = false;
    qDeleteAll(qscene->items());
    qscene->clear();

    fforeach(const PathNode *p, map.robotPath(0)) {
        const Geometry::VisibilityPolygon *vp = p->visibility();
        if(vp) {
            Geometry::VisibilityPolygon rtvp = /*Geometry::Rototranslation(*p) **/ (*vp);
            QList<Geometry::Point> vertices = rtvp.vertices();
            QPolygonF freePoly;
            fforeach(const Geometry::Point &p, vertices) {
                freePoly << QPointF(p.x(), -p.y());
            }
            qscene->addPolygon(freePoly, noPen, QBrush(Qt::white, Qt::SolidPattern))->setZValue(0);
        }
    }

    fforeach(const Geometry::LineSegment *s, map.walls()) {
        qscene->addLine(s->x1(), -s->y1(), s->x2(), -s->y2(), linePen)->setZValue(2);
    }

    if(map.robotPath(0).size() > 1) {
        QList<PathNode *>::const_iterator it = map.robotPath(0).begin(), end = map.robotPath(0).end();
        const PathNode *start = *it; ++it;
        QPointF prev(start->x(), -start->y());
        while(it != end) {
            const PathNode *thisnode = *it;
            QPointF curr(thisnode->x(), -thisnode->y());
            qscene->addLine(QLineF(prev, curr), pointPen)->setZValue(3);
            prev = curr;
            ++it;
        }
    }
    /*
    fforeach(const SLAM::PathNode *node, map.robotPath(0)) {
        static const QPen greenPen(Qt::green);
        static const QBrush greenBrush(Qt::green);
        drawNode(*node, DS_ARROW_LENGTH / 2, greenPen, greenBrush);
    }
    */

    drawNode(*map.lastRobotPose(0), DS_ARROW_LENGTH, pointPen, pointBrush, true, 10);

    update();
    emit zoomChanged(zoom / 10);
}

void MapViewWidget::setScan(const Geometry::PointScan &scan)
{
    static const double dotw = 0.08;
    scanMode = true;
    Geometry::SegmentScan ss(scan, Geometry::SegmentScan::SplitAndMergeInterpolation);
    qDeleteAll(qscene->items());
    qscene->clear();

    drawReferenceFrame();

#ifdef PLOT_POINTS_ELLIPSE
    Geometry::Point centroid = scan.centroid();
    Eigen::Matrix2d cov = scan.sampleCovariance();
    cov(0,1) = cov(1,0) *= -1; // Qt reference frame has y mirrored
    Eigen::Matrix2d covChol = cov.llt().matrixL();

    QGraphicsEllipseItem *ell = new QGraphicsEllipseItem(-3, -3, 6, 6);
    ell->setPen(framePen);
    ell->setBrush(noBrush);
    ell->setTransform(QTransform(
            covChol(0,0), covChol(0,1),
            covChol(1,0), covChol(1,1),
            centroid.x(), -centroid.y()));
    qscene->addItem(ell);
#endif

    QList<Geometry::Point> vertices = ss.toPolygon().vertices();
    QPolygonF freePoly;
    fforeach(const Geometry::Point &p, vertices) {
        freePoly << QPointF(p.x(), -p.y());
    }
    qscene->addPolygon(freePoly, noPen, QBrush(Qt::white, Qt::SolidPattern))->setZValue(0);

    fforeach(const Geometry::Point &p, scan.points()) {
        if(p.norm() < Config::SLAM::laserOutOfRange) {
            qscene->addEllipse(
                        p.x() - dotw / 2, -p.y() - dotw / 2, dotw, dotw,
                        pointPen, pointBrush)->setZValue(2);
        }
    }

    fforeach(const Geometry::LineSegment &s, ss.getSegments()) {
        qscene->addLine(s.x1(), -s.y1(), s.x2(), -s.y2(), linePen)->setZValue(3);
    }

    update();
    centerOn(0, 0);

    emit zoomChanged(zoom / 10);
}

void MapViewWidget::wheelEvent(QWheelEvent *event)
{
    if(event->delta() > 0) {
        if(zoom * zoomMultiplier <= maxZoom) zoom *= zoomMultiplier;
    } else {
        if(zoom / zoomMultiplier >= minZoom) zoom /= zoomMultiplier;
    }
    resetTransform();
    scale(zoom, zoom);
    if(scanMode) centerOn(0, 0);
    emit zoomChanged(zoom / 10);
}

void MapViewWidget::resizeEvent(QResizeEvent *event)
{
    QGraphicsView::resizeEvent(event);
    if(scanMode) centerOn(0, 0);
}

void MapViewWidget::setKeepRatio(bool kr)
{
    keepRatio = kr;
}

int MapViewWidget::heightForWidth(int w) const
{
    if(keepRatio) {
        return w;
    } else {
        return QGraphicsView::heightForWidth(w);
    }
}

void MapViewWidget::reset()
{
    delete qscene;
    qscene = new QGraphicsScene(this);
    setScene(qscene);
    update();
}

double MapViewWidget::scaleValue() const
{
    return zoom * 10;
}

} /* namespace Dataset */
} /* namespace SLAM */
