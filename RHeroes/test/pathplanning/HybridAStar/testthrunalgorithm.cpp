#include "testthrunalgorithm.h"

#include "time.h"
#include "math.h"

#include <QDebug>

namespace Test{

//TestThrunAlgorithm::TestThrunAlgorithm(bool isKenaf)
//{
//    struct tm *current;
//    time_t now;
//    time(&now);
//    current = localtime(&now);
//    srand(time(NULL));

//}

void TestThrunAlgorithm::startSimulation()
{
    //ldbg << "starting simulation" << endl;
    //createUnknownAreas();
    //printUnknownAreas();
    //createWalls();
    loadWalls();
    printWalls();
    /*Data::Pose startingPose(fRand(0,MAX_X), fRand(0,MAX_Y), fRand(-M_PI, M_PI));
    Data::Pose destinationPose(fRand(0,MAX_X), fRand(0,MAX_Y), fRand(-M_PI, M_PI));*/
    /*Data::Pose startingPose(3,1,0);
    Data::Pose destinationPose(10.5,15.5,0);

    printStartDestPoints(startingPose, destinationPose);
    //ldbg << "starting pose is " << startingPose << endl;
    //ldbg << "destination pose is " << destinationPose << endl;
    path = algorithm.calculateNodes(startingPose, destinationPose, walls);
    if(path.size() > 0){        
        printPath();
   }else{
        ldbg << "path not found" << endl;
    }*/
}

void TestThrunAlgorithm::createUnknownAreas()
{
    for(int i=0; i<UNKNOWN_AREAS; i++){
        QPointF topLeft(fRand(0, MAX_X - MAX_SIDE_DIMENSION_UNKNOWN_AREA),fRand(MAX_SIDE_DIMENSION_UNKNOWN_AREA, MAX_Y));
        unknownAreas.append(QRectF(topLeft.x(), topLeft.y(),
                                   fRand(MIN_SIDE_DIMENSION_UNKNOWN_AREA, MAX_SIDE_DIMENSION_UNKNOWN_AREA),
                                   fRand(MIN_SIDE_DIMENSION_UNKNOWN_AREA, MAX_SIDE_DIMENSION_UNKNOWN_AREA)));
    }
}

void TestThrunAlgorithm::createRandomWalls()
{
    QPointF p1;
    QPointF p2;
    for(int i=0; i<N_WALLS; i++){
        bool acceptableWall = true;
        do{
            acceptableWall = true;
            p1 = QPointF(fRand(0,MAX_X), fRand(0,MAX_Y));
            p2 = QPointF(fRand(0,MAX_X), fRand(0,MAX_Y));
            SLAM::Geometry::LineSegment line(p1.x(), p1.y() ,p2.x(),p2.y());
            if(line.length() > MAX_LENGTH_WALL || line.length() < MIN_LENGTH_WALL)
                acceptableWall = false;
            for(int j=0; j < UNKNOWN_AREAS; j++){
                if(unknownAreas.at(j).contains(p1) || unknownAreas.at(j).contains(p2))
                    acceptableWall = false;
            }
        }while(!acceptableWall);
        walls.append(new SLAM::Geometry::LineSegment(p1.x(), p1.y(), p2.x(), p2.y()));
    }
}

double TestThrunAlgorithm::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void TestThrunAlgorithm::printUnknownAreas()
{
}

void TestThrunAlgorithm::printWalls()
{
    QString xIni ="" , yIni="", xFin="", yFin="";
    for(int i=0; i < walls.size(); i++){
        xIni.append(" ").append(QString::number(walls.at(i)->x1(), 'f', 4));
        yIni.append(" ").append(QString::number(walls.at(i)->y1(), 'f', 4));
        xFin.append(" ").append(QString::number(walls.at(i)->x2(), 'f', 4));
        yFin.append(" ").append(QString::number(walls.at(i)->y2(), 'f', 4));
    }
    ldbg << "xWalls=[" << xIni << ";" << xFin << "];" << endl;
    ldbg << "yWalls=[" << yIni << ";" << yFin << "];" << endl;
}


void TestThrunAlgorithm::printStartDestPoints(Data::Pose &startingPose, Data::Pose &destinationPose)
{
    ldbg << "xStartDest=[" << QString::number(startingPose.getX(), 'f', 4) << " " << QString::number(destinationPose.getX(), 'f', 4) << "];" << endl;
    ldbg << "yStartDest=[" << QString::number(startingPose.getY(), 'f', 4) << " " << QString::number(destinationPose.getY(), 'f', 4) << "];" << endl;

}

void TestThrunAlgorithm::printPath()
{
    QString xIni ="" , yIni="", xFin="", yFin="";
    PathPlanner::HybridAStarNode node;
    Data::Pose oldPose;
    Data::Pose newPose;
    node = path.at(0);
    oldPose = node.getPose();
    for(int i=1; i < path.size(); i++){
        node = path.at(i);
        newPose = node.getPose();
        xIni.append(" ").append(QString::number(oldPose.getX(), 'f', 4));
        yIni.append(" ").append(QString::number(oldPose.getY(), 'f', 4));
        xFin.append(" ").append(QString::number(newPose.getX(), 'f', 4));
        yFin.append(" ").append(QString::number(newPose.getY(), 'f', 4));
        oldPose = newPose;
    }
    ldbg << "xPath=[" << xIni << ";" << xFin << "];" << endl;
    ldbg << "yPath=[" << yIni << ";" << yFin << "];" << endl;
}

void TestThrunAlgorithm::loadWalls()
{
    using namespace Data;
    QList<SLAM::Geometry::Point> points;
    points.append(SLAM::Geometry::Point(2,0));
    points.append(SLAM::Geometry::Point(4,0));
    points.append(SLAM::Geometry::Point(4,2));
    points.append(SLAM::Geometry::Point(6,2));
    points.append(SLAM::Geometry::Point(6,1));
    points.append(SLAM::Geometry::Point(5,1));
    points.append(SLAM::Geometry::Point(5,0));
    points.append(SLAM::Geometry::Point(8,0));
    points.append(SLAM::Geometry::Point(8,4));
    points.append(SLAM::Geometry::Point(9,4));
    points.append(SLAM::Geometry::Point(9,2));
    points.append(SLAM::Geometry::Point(10,2));
    points.append(SLAM::Geometry::Point(10,5));
    points.append(SLAM::Geometry::Point(14,5));
    points.append(SLAM::Geometry::Point(14,10));
    points.append(SLAM::Geometry::Point(15,10));
    points.append(SLAM::Geometry::Point(15,5));
    points.append(SLAM::Geometry::Point(16,5));
    points.append(SLAM::Geometry::Point(16,7));
    points.append(SLAM::Geometry::Point(19,7));
    points.append(SLAM::Geometry::Point(19,8));
    points.append(SLAM::Geometry::Point(16,8));
    points.append(SLAM::Geometry::Point(16,11));
    points.append(SLAM::Geometry::Point(17,11));
    points.append(SLAM::Geometry::Point(17,12));
    points.append(SLAM::Geometry::Point(15,12));
    points.append(SLAM::Geometry::Point(15,16));
    points.append(SLAM::Geometry::Point(7,16));
    points.append(SLAM::Geometry::Point(7,14));
    points.append(SLAM::Geometry::Point(5,14));
    points.append(SLAM::Geometry::Point(5,11));
    points.append(SLAM::Geometry::Point(0,11));
    points.append(SLAM::Geometry::Point(0,3));
    points.append(SLAM::Geometry::Point(2,3));

    createWalls(points);


    points.clear();
    points.append(SLAM::Geometry::Point(1,4));
    points.append(SLAM::Geometry::Point(7,4));
    points.append(SLAM::Geometry::Point(7,5));
    points.append(SLAM::Geometry::Point(3,5));
    points.append(SLAM::Geometry::Point(3,7));
    points.append(SLAM::Geometry::Point(7,7));
    points.append(SLAM::Geometry::Point(7,12));
    points.append(SLAM::Geometry::Point(8,12));
    points.append(SLAM::Geometry::Point(8,13));
    points.append(SLAM::Geometry::Point(6,12));
    points.append(SLAM::Geometry::Point(6,9));
    points.append(SLAM::Geometry::Point(1,9));

    createWalls(points);


    points.clear();
    points.append(SLAM::Geometry::Point(10,7));
    points.append(SLAM::Geometry::Point(13,7));
    points.append(SLAM::Geometry::Point(13,9));
    points.append(SLAM::Geometry::Point(12,9));
    points.append(SLAM::Geometry::Point(12,10));
    points.append(SLAM::Geometry::Point(13,10));
    points.append(SLAM::Geometry::Point(13,12));
    points.append(SLAM::Geometry::Point(14,12));
    points.append(SLAM::Geometry::Point(14,15));
    points.append(SLAM::Geometry::Point(8,15));
    points.append(SLAM::Geometry::Point(8,14));
    points.append(SLAM::Geometry::Point(9,14));
    points.append(SLAM::Geometry::Point(9,12));
    points.append(SLAM::Geometry::Point(10,12));
    points.append(SLAM::Geometry::Point(10,11));
    points.append(SLAM::Geometry::Point(8,11));
    points.append(SLAM::Geometry::Point(8,9));
    points.append(SLAM::Geometry::Point(10,9));

    createWalls(points);
}

void TestThrunAlgorithm::createWalls(QList<SLAM::Geometry::Point> points)
{
    int i;
    for(i=1; i < points.size(); i++){
        walls.append(new SLAM::Geometry::LineSegment(points.at(i-1),points.at(i)));
    }
    walls.append(new SLAM::Geometry::LineSegment(points.at(i-1),points.at(0)));
}
}

