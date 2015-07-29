#include "testevaluationprm.h"
#include "PRM/prmalgorithm.h"
#include "PRM/evaluationPRM/prmfunction.h"
#include "PRM/evaluationPRM/prmfunctionmcdm.h"
#include "PRM/evaluationPRM/prmfunctiontovar.h"
#include "PRM/evaluationPRM/criteriaPRM/informationgaincriterionprm.h"

namespace TestPRM{

using namespace SLAM;
using namespace Data;
using namespace PRM;
using namespace SLAM::Geometry;

TestEvaluationPRM::TestEvaluationPRM(QObject *parent) :
    QObject(parent)
{
}

void TestEvaluationPRM::testEval(){
    ldbg << "TestEvaluationPRM: testEvaluation" << endl;
    PRMAlgorithm* prm = new PRMAlgorithm("TestEvaluationPRM_testEvaluation");
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
    Frontier f1 = Frontier(Point(20,20),Point(-20,20));
    map.addFrontier(f);
    map.addFrontier(f1);
    prm->updatePRM(map);
    prm->plot(map);
    PathPlannerPRM* planner=new PathPlannerPRM();
    PRMFunction function= PRMFunction(prm, planner,0);
    QList<Frontier*> listFrontiersEval;
    QHash<uint,double>* signal= new QHash<uint,double>();
    listFrontiersEval<<&f<<&f1;
    EvaluationRecords* value=function.evaluateFrontiers(listFrontiersEval,map,100,*signal);
    ldbg << "TestEvaluationPRM: evaluation: "<< endl;
    foreach(Frontier f, *(value->getEvaluatedFrontiers())){
        double v = value->getEvaluation(f);
        ldbg<<"Frontier: "<<f.centroid()<<" value: "<<v<<endl;
    }
}

void TestEvaluationPRM::testEval2(){
    ldbg << "TestEvaluationPRM: testEvaluation2" << endl;
    PRMAlgorithm* prm = new PRMAlgorithm("TestEvaluationPRM_testEvaluation2");
    Map map = Map();
    TimedPose pose = TimedPose(0,Pose(1,1,0));
    map.addPose(Config::robotID,pose);
    QList<Point> points;
    QList<LineSegment*> walls;
    points.append(Point(2,10));
    points.append(Point(2,0));
    points.append(Point(0,0));
    points.append(Point(0,8));
    int i;
    for(i=1; i<points.size();i++){
        walls.append(new LineSegment(points.at(i-1),points.at(i)));
    }
    walls.append(new LineSegment(Point(0,10), Point(0,20)));
    walls.append(new LineSegment(Point(0,20), Point(4,20)));
    walls.append(new LineSegment(Point(4,18), Point(2,18)));
    walls.append(new LineSegment(Point(2,18), Point(2,12)));
    foreach(LineSegment* w,walls){
        map.addWall(*w);
    }
    Frontier f = Frontier(Point(4,20),Point(4,18));
    Frontier f1 = Frontier(Point(0,8),Point(0,10));
    Frontier f2 = Frontier(Point(2,10),Point(2,12));
    map.addFrontier(f);
    map.addFrontier(f1);
    map.addFrontier(f2);
    prm->updatePRM(map);
    prm->plot(map);
    PathPlannerPRM* planner= new PathPlannerPRM();
    PRMFunction function= PRMFunction(prm, planner,0);
    PRMFunctionMCDM functionMCDM= PRMFunctionMCDM(prm, planner,0);
    PRMFunctionTovar functionTovar= PRMFunctionTovar(prm, planner,0);
    QList<Frontier*> listFrontiersEval;
    QHash<uint,double>* signal= new QHash<uint,double>();
    listFrontiersEval<<&f<<&f1<<&f2;
    function.evaluateFrontiers(listFrontiersEval,map,100,*signal);
    functionMCDM.evaluateFrontiers(listFrontiersEval,map,100,*signal);
    functionTovar.evaluateFrontiers(listFrontiersEval,map,100,*signal);
}


}
