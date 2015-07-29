/*
 * exporter.cpp
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#include "exporter.h"
#include "mapviewwidget.h"
#include "datasetworker.h"
#include "slam/utilities.h"
#include <QFile>
#include <QFileInfo>
#include <QPrinter>
#include <QPainter>
#include <QAbstractGraphicsShapeItem>
#include <QTextStream>
#include <QtAlgorithms>

namespace SLAM {
namespace Dataset {

void Exporter::exportPDF(const QString &file, const MapViewWidget *map)
{
    double scale = map->scaleValue() / 5;
    QPrinter printer;
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(file);
    QPainter pdfPainter(&printer);
    QGraphicsScene scene;
    //printer.setPageMargins(0, 0, 0, 0, QPrinter::DevicePixel);
    pdfPainter.setBackground(map->backgroundBrush());
    scene.setBackgroundBrush(map->backgroundBrush());
    QList<QGraphicsItem *> items = map->scene()->items();
    fforeach(QGraphicsItem *it, items) {
        QGraphicsLineItem *line = dynamic_cast<QGraphicsLineItem *>(it);
        QGraphicsEllipseItem *ell = dynamic_cast<QGraphicsEllipseItem *>(it);
        QGraphicsPolygonItem *poly = dynamic_cast<QGraphicsPolygonItem *>(it);
        if(line) {
            QPen p = line->pen();
            p.setWidthF(p.widthF() / scale);
            scene.addLine(line->line(), p)->setZValue(line->zValue());
        } else if(ell) {
            if(ell->rect().width() > 80) continue;
            QPen p = ell->pen();
            p.setWidthF(p.widthF() / scale);
            QGraphicsEllipseItem *newEllipse = scene.addEllipse(ell->rect(), p, ell->brush());
            newEllipse->setTransform(ell->transform());
            newEllipse->setZValue(ell->zValue());
        } else if(poly) {
            QPen p = poly->pen();
            p.setWidthF(p.widthF() / scale);
            scene.addPolygon(poly->polygon(), p, poly->brush())->setZValue(poly->zValue());
        }
    }

    scene.render(&pdfPainter);
    pdfPainter.end();
    qDeleteAll(scene.items());
}

void Exporter::exportCPPCode(const QString &file, DatasetWorker *worker)
{
    QFile f(file);
    if(f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        double maxts = 0;
        QTextStream cpp(&f);
        Map m = worker->previousMap();
        Geometry::PointScan s = worker->scan();
        Data::Pose delta = worker->deltaOdometry();

        QString ifndefName = QFileInfo(f).fileName();

        fforeach(QChar &c, ifndefName) {
            if(c.isLetterOrNumber()) {
                c = c.toUpper();
            } else {
                c = '_';
            }
        }
        ifndefName.append('_');
        if(ifndefName[0].isNumber()) {
            ifndefName.prepend('_');
        }


        cpp.setRealNumberPrecision(16);
        cpp <<
                "#ifndef " << ifndefName << "\n"
                "#define " << ifndefName << "\n"
                "/* Dataset: " << worker->datasetFileName() << " */\n"
                "/* Iteration: " << worker->iteration() << " */\n"
                "\n"
                "#include \"data/pose.h\"\n"
                "#include \"slam/map.h\"\n"
                "#include \"slam/geometry/pointscan.h\"\n"
                "\n"
                "inline SLAM::Map getDatasetMap() {\n"
                "    SLAM::Map ret;\n";

        fforeach(const Geometry::LineSegment *l, m.walls()) {
            cpp <<
                    "    ret.addWall(SLAM::Geometry::LineSegment(" <<
                    l->x1() << ", " << l->y1() << ", " << l->x2() << ", " << l->y2() << "));\n";
        }
        fforeach(const Geometry::Frontier *l, m.frontiers()) {
            cpp <<
                    "    ret.addFrontier(SLAM::Geometry::Frontier(" <<
                    l->x1() << ", " << l->y1() << ", " << l->x2() << ", " << l->y2() << "));\n";
        }
        foreach(uint r, m.knownRobots()) {
            fforeach(const PathNode *p, m.robotPath(r)) {
                maxts = std::max(maxts, p->timestamp());
                cpp <<
                        "    ret.addPose(" << r << ", SLAM::TimedPose(" << p->timestamp() <<
                        ", Data::Pose(" << p->x() << ", " << p->y() << ", " << p->theta() <<
                        ")));\n";
            }
        }

        cpp <<
                "    return ret;\n"
                "}\n"
                "\n"
                "inline SLAM::Geometry::PointScan getDatasetScan() {\n"
                "    QList<double> readings;\n";

        fforeach(double r, s.readings()) {
            cpp << "    readings.append(" << r << ");\n";
        }
        cpp <<
                "    Data::LaserData laser(" << maxts << ", " << s.fov() << ", " <<
                    s.resolution() << ", readings);\n"
                "    return SLAM::Geometry::PointScan(laser);\n"
                "}\n"
                "\n"
                "inline Data::Pose getDatasetDeltaOdometry() {\n"
                "    return Data::Pose(" << delta.x() << "," << delta.y() <<
                    "," << delta.theta() << ");\n"
                "}\n"
                "\n"
                "#endif /* " << ifndefName << " */";

        f.close();
    }
}

} /* namespace Dataset */
} /* namespace SLAM */
