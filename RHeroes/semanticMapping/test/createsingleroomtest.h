#ifndef CREATESINGLEROOMTEST_H
#define CREATESINGLEROOMTEST_H

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
class CreateSingleRoomTest
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
    CreateSingleRoomTest();
    void WallClusterHandler( QList<secondWallStruct> list );
    QList<Geometry::DoorLineSegment> doors;
    void CreateRooms();
    void CreateSingleRoom(SLAM::Geometry::Point p);
    QList<Room> rooms;
    double maxX, maxY, minX, minY;
    QList<SLAM::Geometry::LineSegment> walls;
};
    }
}


#endif // CREATESINGLEROOMTEST_H
