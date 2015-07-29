#ifndef CLUSTERADDTEST_H
#define CLUSTERADDTEST_H
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
class ClusterAddTest
{
public:
    typedef struct initWallStruct
    {
        double angle;
        QList<SLAM::Geometry::LineSegment> line;
    } initWallStruct;
    typedef struct secondWallStruct
    {
        double M, Q, QM;
        QList<SLAM::Geometry::LineSegment> line;
    } secondWallStruct;
    ClusterAddTest();
};
    }
}

#endif // CLUSTERADDTEST_H
