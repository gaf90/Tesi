#include "createroomtest.h"
#include "semanticMapping/SemanticDef.h"
#include <QDebug>
#include "slam/geometry/point.h"

namespace SemanticMapping{
    namespace Test {

CreateRoomTest::CreateRoomTest()
{
    QList<SLAM::Geometry::Point> initPoints;
    double maxX, maxY, minX, minY;
    maxX = 12;
    maxY = 3.142957;
    minX = -2;
    minY = 0;
    qDebug() << "{maxX, maxY, minX, minY} = { " << maxX << ", " <<
                maxY << ", " << minX << ", " << minY << " }" << endl;
    int numberOfPoints = (int)  (( maxX - minX ) * (maxY - minY) / POINTS_PER_SQUARE_METER );
    for (int i=0; i < numberOfPoints; i++)
    {
        initPoints.append( * new SLAM::Geometry::Point(
                                (double) rand() / (RAND_MAX + 1.0) * (maxX - minX) + minX ,
                                (double) rand() / (RAND_MAX + 1.0) * (maxY - minY) + minY  ));
    }
    for (int i=0; i < numberOfPoints; i++) qDebug() << "il punto che sto aggiungendo vale (" << initPoints[i].x()
                                                    << "," << initPoints[i].y() << ")" <<endl;
}
    }
}
