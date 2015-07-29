#include "testintersect.h"
#include "slam/geometry/linesegment.h"
#include "slam/geometry/point.h"
#include <QMap>
#include <QDebug>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math.h"

#define NUMBER_OF_POINTS 10000
#define NUMBER_OF_LINES 1000
#define NUMBER_OF_ROUNDS 1
#define PERCENTAGE_NEEDED_TO_FIND_SOLUTION 0.01
#define MAX_X 50
#define MAX_Y 50
#define MAX_DISTANCE 5

#define NUMBER_OF_ROBOTS 4
#define NUMBER_OF_MOVEMENTS 30
#define MAX_MOVEMENT_DISTANCE 5
#define NUMBER_OF_WALLS 500
#define MAX_WALL_LENGTH 5
#define POINTS_PER_METER 1
#define MAX_DISTANCE_TO_CREATE_LINK 5
#define NUMBER_OF_ITERATIONS 3

#define USE_DISTANCE

namespace Test{
TestIntersect::TestIntersect()
{
}

void TestIntersect::doSpeedTest()
{
    struct tm *current;
    time_t now;

    time(&now);
    current = localtime(&now);
    srand(time(NULL));

    qDebug() << "the initial time is "<< current->tm_hour << ":" << current->tm_min << ":" << current->tm_sec;


    QList<SLAM::Geometry::LineSegment> lines;
    QList<SLAM::Geometry::Point> points;
    SLAM::Geometry::Point tempPoint;
    SLAM::Geometry::LineSegment tempLine;
    SLAM::Geometry::LineSegment path;
    for(int i=0; i<NUMBER_OF_POINTS; i++){
        tempPoint = SLAM::Geometry::Point(fRand(0,MAX_X), fRand(0,MAX_Y));
        points.append(tempPoint);
    }
    for(int i=0; i<NUMBER_OF_LINES; i++){
        tempLine = SLAM::Geometry::LineSegment(fRand(0,MAX_X), fRand(0,MAX_Y),fRand(0,MAX_X), fRand(0,MAX_Y));
        lines.append(tempLine);
    }

    QMap<double, SLAM::Geometry::Point *> orderedPoints;    

    time(&now);
    current = localtime(&now);

    qDebug() << "starting at: " << current->tm_hour << ":" << current->tm_min << ":" << current->tm_sec;

    SLAM::Geometry::Point destinationPoint = SLAM::Geometry::Point(fRand(0,MAX_X), fRand(0,MAX_Y));;
    foreach(tempPoint, points){
        orderedPoints.insert(destinationPoint.distance(tempPoint), &tempPoint);
    }

#ifndef USE_DISTANCE
    for(int i=0; i<PERCENTAGE_NEEDED_TO_FIND_SOLUTION*NUMBER_OF_POINTS; i++){
        path = SLAM::Geometry::LineSegment(points.at(i), destinationPoint);
        foreach(tempLine, lines){
            tempLine.intersects(path,0.01);
        }
    }
#else
    int count = 0;
    QMapIterator<double, SLAM::Geometry::Point *> i(orderedPoints);
    while (i.hasNext()) {
         i.next();
         if(i.key() > MAX_DISTANCE)
             break;
         path = SLAM::Geometry::LineSegment(*i.value(), destinationPoint);
         foreach(tempLine, lines){
             tempLine.intersects(path,0.01);
         }
        count++;
     }
    qDebug() << "counted " << count << "elements";
#endif

    time(&now);
    current = localtime(&now);

    qDebug() << "concluding at "<< current->tm_hour << ":" << current->tm_min << ":" << current->tm_sec;
}

void TestIntersect::doPlannerSimulationTest()
{
    //crea muri
//    for numero di iterazioni
//            crea azioni per ogni robot
//            per ogni robot
//                con i robot successivi crea link
//            stampa
}


double TestIntersect::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


}


