#include "testprmalgorithm.h"
#include "PRM/prmalgorithm.h"
#include <QDebug>

namespace TestPRM{

using namespace SLAM;
using namespace Data;
using namespace SLAM::Geometry;
using namespace PRM;

TestPRMAlgorithm::TestPRMAlgorithm(QObject *parent): QObject(parent)
{

}

TestPRMAlgorithm::~TestPRMAlgorithm()
{

}

void TestPRMAlgorithm::testRandomPoint(){
    //qDebug()<<"Inizio test random";
    int iteration=100;
    //punto robot
    PRMAlgorithm* prm = new PRMAlgorithm();
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(0,0,0));
    map.addPose(Config::robotID,pose);
    for(int i=0; i<iteration;i++){
        Point* point = prm->newRandomPoint(map);
        //qDebug()<<"Coordinate del punto: " << point.x() << "," << point.y();
        QVERIFY2(sqrt((point->x()*point->x())+(point->y()*point->y()))<=Config::PRM::maxPointDistance,"punto non valido, troppo lontano dal robot");
    }
    //frontiera orizzontale sopra
    Frontier* f = new Frontier(-2,-2,2,-2);
    map.addFrontier(*f);
    for(int i=0; i<iteration;i++){
        Point* point2 = prm->newRandomPointFrontier(f,map,True);
        //qDebug()<<"Coordinate punto frontiera: " << *point2.x() << "," << point2.y();
        QVERIFY2(point2->y()>=-2,"punto frontiera orizzontale sopra non valido");
    }

    //frontiera orizzontale sotto
    Frontier* f4 = new Frontier(-2,2,2,2);
    map.addFrontier(*f4);
    for(int i=0; i<iteration;i++){
        Point* point5 = prm->newRandomPointFrontier(f4,map,False);
        //        qDebug()<<"Coordinate punto frontiera: " << *point5;
        QVERIFY2(point5->y()<=2,"punto frontiera orizzontale sotto non valido");
    }

    //frontiera verticale sinistra
    Frontier* f2 = new Frontier(2,-2,2,2);
    map.addFrontier(*f2);
    for(int i=0; i<iteration;i++){
        Point* point3 = prm->newRandomPointFrontier(f2, map,True);
        //qDebug()<<"Coordinate punto frontiera: " << *point3.x() << "," << point3.y();
        QVERIFY2(point3->x()<=2,"punto frontiera verticale sinistra non valido");
    }

    //frontiera verticale destra
    Frontier* f3 = new Frontier(-2,-2,-2,2);
    map.addFrontier(*f3);
    for(int i=0; i<iteration;i++){
        Point* point4 = prm->newRandomPointFrontier(f3,map,False);
        //qDebug()<<"Coordinate punto frontiera: " << *point4.x() << "," << point4.y();
        QVERIFY2(point4->x()>=-2,"punto frontiera verticale destra non valido");
    }
    //qDebug()<<"Fine test random";
    //QVERIFY(true==false);
}


void TestPRMAlgorithm::testVisibilityCheckNewMap(){
    //qDebug()<<"Inizio test Visibility";
    PRMAlgorithm* prm = new PRMAlgorithm();
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
    walls.append(new LineSegment(points.at(i-1),points.at(0)));
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    //controllo punto valido
    Point* p= new Point(5,5);
    bool res=prm->visibilityCheckNewMap(p,map);
    QVERIFY2(res==true,"punto non valido");
    //controllo punto non valido
    Point* p1= new Point(11,11);
    res=prm->visibilityCheckNewMap(p1,map);
    QVERIFY2(res==false,"punto non valido");
    //qDebug()<<"Fine test Visibility";
}

void TestPRMAlgorithm::testVisibilityCheckFrontierNotPassed(){
    //controllo validità punti rispetto a una frontiera non oltrepassata
    PRMAlgorithm* prm = new PRMAlgorithm();
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(2,2,0));
    map.addPose(Config::robotID,pose);
    QList<Point> points;
    QList<LineSegment*> walls;
    points.append(Point(5,0));
    points.append(Point(0,0));
    points.append(Point(0,5));
    points.append(Point(5,5));
    int i;
    for(i=1; i<points.size();i++){
        walls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(5,0),Point(5,5));
    map.addFrontier(f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==3);
    prm->oldMap=map;
    prm->frontiersRemoved.append(f);
    //creazione mappa quattro muri
    Map newMap = Map();
    TimedPose newPose = TimedPose(0,Pose(4,2,0));
    newMap.addPose(Config::robotID,pose);
    newMap.addPose(Config::robotID,newPose);
    QList<Point> newPoints;
    newPoints.append(Point(5,5));
    newPoints.append(Point(10,5));
    newPoints.append(Point(10,0));
    newPoints.append(Point(5,0));
    for(i=1; i<newPoints.size();i++){
        walls.append(new LineSegment(newPoints.at(i-1),newPoints.at(i)));
    }
    foreach(LineSegment* w,walls){
        newMap.addWall(*w);
    }
    QVERIFY(newMap.frontiers().size()==0);
    QVERIFY(newMap.walls().size()==6);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->x()==4);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->y()==2);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->previous()->x()==2);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->previous()->y()==2);
    Point* p1= new Point(6,3);//valido è nella nuova area oltre la frontiera non oltrepassata
    Point* p2= new Point(4,4);//non valido è nella vecchia area
    Point* p3= new Point(11,11);//non valido è totalmente fuori dalla mappa
    Point* p4= new Point(5,2);//il nuovo punto è esattamente sulla frontiera deve fallire
    bool res1=prm->visibilityCheck(p1,newMap);
    QVERIFY2(res1==true,"punto non valido");
    bool res2=prm->visibilityCheck(p2,newMap);
    QVERIFY2(res2==false,"punto non valido");
    bool res3=prm->visibilityCheck(p3,newMap);
    QVERIFY2(res3==false,"punto non valido");
    bool res4=prm->visibilityCheck(p4,newMap);
    QVERIFY2(res4==true,"punto non valido");
}

void TestPRMAlgorithm::testVisibilityCheckFrontierPassed(){
    //controllo validità punti rispetto ad una frontiera oltrepassata
    PRMAlgorithm* prm = new PRMAlgorithm();
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(2,2,0));
    map.addPose(Config::robotID,pose);
    QList<Point> points;
    QList<LineSegment*> walls;
    points.append(Point(5,0));
    points.append(Point(0,0));
    points.append(Point(0,5));
    points.append(Point(5,5));
    int i;
    for(i=1; i<points.size();i++){
        walls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(5,0),Point(5,5));
    map.addFrontier(f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==3);
    prm->oldMap=map;
    prm->frontiersRemoved.append(f);
    //creazione mappa quattro muri
    Map newMap = Map();
    TimedPose newPose = TimedPose(0,Pose(7,2,0));
    newMap.addPose(Config::robotID,pose);
    newMap.addPose(Config::robotID,newPose);
    QList<Point> newPoints;
    newPoints.append(Point(5,5));
    newPoints.append(Point(10,5));
    newPoints.append(Point(10,0));
    newPoints.append(Point(5,0));
    for(i=1; i<newPoints.size();i++){
        walls.append(new LineSegment(newPoints.at(i-1),newPoints.at(i)));
    }
    foreach(LineSegment* w,walls){
        newMap.addWall(*w);
    }
    QVERIFY(newMap.frontiers().size()==0);
    QVERIFY(newMap.walls().size()==6);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->x()==7);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->y()==2);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->previous()->x()==2);
    QVERIFY(newMap.lastRobotPose(Config::robotID)->previous()->y()==2);
    Point* p1= new Point(6,3);//valido è nella nuova area oltre la frontiera
    Point* p2= new Point(4,4);//non valido è nella vecchia area dietro la frontiera
    Point* p3= new Point(11,11);//non valido è totalmente fuori dalla mappa
    Point* p4= new Point(5,2);//il nuovo punto è esattamente sulla frontiera deve fallire
    bool res1=prm->visibilityCheck(p1,newMap);
    QVERIFY2(res1==true,"punto non valido");
    bool res2=prm->visibilityCheck(p2,newMap);
    QVERIFY2(res2==false,"punto non valido");
    bool res3=prm->visibilityCheck(p3,newMap);
    QVERIFY2(res3==false,"punto non valido");
    bool res4=prm->visibilityCheck(p4,newMap);
    QVERIFY2(res4==false,"punto non valido");
}




void TestPRMAlgorithm::testVisibilityCheckFrontier(){
    //test validità punti frontiera 5 muri e una frontiera
    PRMAlgorithm* prm = new PRMAlgorithm();
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(0,0,0));
    map.addPose(Config::robotID,pose);
    QList<Point> points;
    QList<LineSegment*> walls;
    points.append(Point(0,0));
    points.append(Point(0,5));
    points.append(Point(5,5));
    points.append(Point(5,10));
    points.append(Point(10,10));
    points.append(Point(10,0));
    int i;
    for(i=1; i<points.size();i++){
        walls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(0,0),Point(10,0));
    map.addFrontier(f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==5);
    Point* p1= new Point(5,2);//valido
    Point* p2= new Point(6,9);//vede solo uno
    Point* p3= new Point(5,11);//non valido
    bool res1=prm->visibilityCheckFrontier(p1,&f,map);
    QVERIFY2(res1==true,"punto non valido");
    bool res2=prm->visibilityCheckFrontier(p2,&f,map);
    QVERIFY2(res2==true,"punto non valido");
    bool res3=prm->visibilityCheckFrontier(p3,&f,map);
    QVERIFY2(res3==false,"punto non valido");

}


void TestPRMAlgorithm::testFoundFrontiers(){
    //qDebug()<<"Inizio test Visibility";
    //creazione mappa tre muri e una frontiera
    PRMAlgorithm* prm = new PRMAlgorithm();
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
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(10,10),Point(-10,10));
    map.addFrontier(f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==3);
    Point* p= new Point(0,11);
    bool res=prm->visibilityCheckNewMap(p,map);
    QVERIFY2(res==false,"punto non valido");
    prm->oldMap=map;
    //creazione mappa quattro muri
    Map newMap = Map();
    QList<LineSegment*> newWalls;
    for(i=1; i<points.size();i++){
        newWalls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    newWalls.append(new LineSegment(points.at(i-1),points.at(0)));
    //qDebug()<<"nMuri"<<newWalls.size();
    foreach(LineSegment* w,newWalls){
        newMap.addWall(*w);
    }
    QVERIFY(newMap.frontiers().size()==0);
    QVERIFY(newMap.walls().size()==4);
    QList<Frontier> frontiersRemoved = prm->foundFrontiersRemoved(newMap);
    QVERIFY(frontiersRemoved.size()==1);
    QList<Frontier> frontiersAdded = prm->foundFrontiersAdded(newMap);
    QVERIFY(frontiersAdded.size()==0);
    //controllo mappa due frontiere e due muri
    Map newMap2 = Map();
    QList<LineSegment*> newWalls2;
    for(i=1; i<points.size()-1;i++){
        newWalls2.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    foreach(LineSegment* w,newWalls2){
        newMap2.addWall(*w);
    }
    Frontier f2 = Frontier(Point(-10,-10),Point(-10,10));
    newMap2.addFrontier(f2);
    newMap2.addFrontier(f);
    QVERIFY(newMap2.frontiers().size()==2);
    QVERIFY(newMap2.walls().size()==2);
    QList<Frontier> frontiersRemoved2 = prm->foundFrontiersRemoved(newMap2);
    QVERIFY(frontiersRemoved2.size()==0);
    QList<Frontier> frontiersAdded2 = prm->foundFrontiersAdded(newMap2);
    QVERIFY(frontiersAdded2.size()==1);
}

void TestPRMAlgorithm::testPRMUpdate(){
    PRMAlgorithm* prm = new PRMAlgorithm("TestPRMAlgorithm_testPRMUpdate");
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
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(10,10),Point(-10,10));
    map.addFrontier(f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==3);
    prm->updatePRM(map);
    QVERIFY(prm->iterationNumber==1);
    int point= prm->graph.nodes.size();
    int frontierPoint= prm->graph.frontierNodes.size();
    int edge= prm->graph.edges.size();
    //qDebug()<<"numero di punti: "<<point;
    //qDebug()<<"numero di punti frontiera: "<<frontierPoint;
    //qDebug()<<"numero di archi: "<<edge;
    //aggiornamento mappa
    Map newMap = Map();
    TimedPose newPose = TimedPose(0,Pose(0,9,0));
    newMap.addPose(Config::robotID,pose);
    newMap.addPose(Config::robotID,newPose);
    QList<Point> newPoints;
    newPoints.append(Point(-10,10));
    newPoints.append(Point(-10,20));
    newPoints.append(Point(10,20));
    for(i=1; i<newPoints.size();i++){
        walls.append(new LineSegment(newPoints.at(i-1),newPoints.at(i)));
    }
    foreach(LineSegment* w,walls){
        newMap.addWall(*w);
    }
    Frontier f1 = Frontier(Point(10,10),Point(10,20));
    newMap.addFrontier(f1);
    QVERIFY(newMap.frontiers().size()==1);
    QVERIFY(newMap.walls().size()==5);
    prm->updatePRM(newMap);
    point= prm->graph.nodes.size();
    frontierPoint= prm->graph.frontierNodes.size();
    edge= prm->graph.edges.size();
    //qDebug()<<"numero di punti: "<<point;
    //qDebug()<<"numero di punti frontiera: "<<frontierPoint;
    //qDebug()<<"numero di archi: "<<edge;


    QTest::qSleep(20*1000);
}
}


