#ifndef WALLCLUSTERHANDLERTEST_H
#define WALLCLUSTERHANDLERTEST_H
#include <QtCore/QCoreApplication>
#include "math.h"
#include "iostream"
#include "cmath"
#include <QPointF>
#include "ostream"
#include "semanticMapping/room.h"
#include "semanticMapping/semantichandler.h"
#include "slam/geometry/linesegment.h"
#include "slam/map.h"

namespace SemanticMapping{
    namespace Test{
class WallClusterHandlerTest
{
public:
    typedef struct initWallStruct
    {
        double angle;
        QList<SLAM::Geometry::LineSegment> line;
        bool orizontal, vertical;
    } initWallStruct;
    typedef struct secondWallStruct
    {
        double Q, M, QM;
        QList<SLAM::Geometry::LineSegment> line;
    } secondWallStruct;
    WallClusterHandlerTest();
    void WallClusterHandler( QList<secondWallStruct> list );
    QList<Geometry::DoorLineSegment> doors;
};
    }
}


#endif // WALLCLUSTERHANDLERTEST_H
