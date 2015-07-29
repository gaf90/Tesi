#include "testastarprm.h"
#include "PRM/prmalgorithm.h"


namespace TestPRM{

using namespace SLAM;
using namespace Data;
using namespace PRM;
using namespace SLAM::Geometry;

TestAStarPRM::TestAStarPRM(QObject *parent) :
    QObject(parent)
{
}

void TestAStarPRM::testPath(){
    ldbg << "AStarPRM: testPath" << endl;
    PRMAlgorithm* prm = new PRMAlgorithm("AStarPRM_testPath");
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(0,0,0));
    map.addPose(Config::robotID,pose);
    QList<Point> points;
    QList<LineSegment*> walls;
    points.append(Point(10,10));
    points.append(Point(10,-10));
    points.append(Point(-10,-10));
    points.append(Point(-10,10));
    int i;
    for(i=1; i<points.size();i++){
        walls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    walls.append(new LineSegment(Point(-2,2), Point(-1,2)));
    walls.append(new LineSegment(Point(1,2), Point(2,2)));
    walls.append(new LineSegment(Point(-4,5), Point(-2,5)));
    walls.append(new LineSegment(Point(2,7), Point(4,7)));
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(10,10),Point(-10,10));
    map.addFrontier(f);
    prm->updatePRM(map);
    ldbg << "AStarPRM: updatePRM" << endl;
    prm->getPaths(new Point(0,10));
}

void TestAStarPRM::testPath2(){
    ldbg << "AStarPRM: testPath2" << endl;
    PRMAlgorithm* prm = new PRMAlgorithm("AStarPRM_testPath2");
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(0,8,0));
    map.addPose(Config::robotID,pose);
    QList<LineSegment*> walls;
    walls.append(new LineSegment(Point(-10,10), Point(-10,0)));
    walls.append(new LineSegment(Point(10,10), Point(10,0)));
    walls.append(new LineSegment(Point(-5,0), Point(5,0)));
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(10,10),Point(-10,10));
    Frontier f2 = Frontier(Point(-10,0),Point(-5,0));
    Frontier f3 = Frontier(Point(10,0),Point(5,0));
    map.addFrontier(f);
    map.addFrontier(f2);
    map.addFrontier(f3);
    prm->updatePRM(map);
    //aggiornamento mappa
    Map newMap = Map();
    TimedPose newPose = TimedPose(0,Pose(-8,-1,0));
    newMap.addPose(Config::robotID,pose);
    newMap.addPose(Config::robotID,newPose);
    QList<LineSegment*> newWalls;
    newWalls.append(new LineSegment(Point(-10,10), Point(-10,-10)));
    newWalls.append(new LineSegment(Point(-10,-10), Point(-5,-10)));
    newWalls.append(new LineSegment(Point(-5,0), Point(5,0)));
    newWalls.append(new LineSegment(Point(-5,0), Point(-5,-2)));
    newWalls.append(new LineSegment(Point(10,10), Point(10,0)));
//    newWalls.append(new LineSegment(Point(-5,-7), Point(5,-7)));
//    newWalls.append(new LineSegment(Point(5,-7), Point(5,-5)));
    foreach(LineSegment* w,newWalls){
        newMap.addWall(*w);
    }
    Frontier f4 = Frontier(Point(-5,-10),Point(-5,-2));
    newMap.addFrontier(f);
    newMap.addFrontier(f3);
    newMap.addFrontier(f4);
    prm->updatePRM(newMap);
    prm->getPaths(new Point(0,10));
    //aggiornamento mappa
    Map newMap2 = Map();
    TimedPose newPose2 = TimedPose(0,Pose(0,-9,0));
    newMap2.addPose(Config::robotID,pose);
    newMap2.addPose(Config::robotID,newPose);
    newMap2.addPose(Config::robotID,newPose2);
    QList<LineSegment*> newWalls2;
    newWalls2.append(new LineSegment(Point(-10,10), Point(-10,-10)));
    newWalls2.append(new LineSegment(Point(-10,-10), Point(10,-10)));
    newWalls2.append(new LineSegment(Point(10,10), Point(10,-10)));

    newWalls2.append(new LineSegment(Point(-5,0), Point(5,0)));
    newWalls2.append(new LineSegment(Point(-5,0), Point(-5,-2)));
    newWalls2.append(new LineSegment(Point(-5,-2), Point(5,-2)));
    newWalls2.append(new LineSegment(Point(5,-2), Point(5,0)));
    foreach(LineSegment* w,newWalls2){
        newMap2.addWall(*w);
    }
    newMap2.addFrontier(f);
    prm->updatePRM(newMap2);
    prm->getPaths(new Point(0,10));
    //plot edges
//    prm->graph.updateEdges(newMap2);
//    prm->plot(newMap2);
}


}


