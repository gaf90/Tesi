#include "testgraph.h"
#include <QDebug>
#include "PRM/graph.h"
#include "shared/config.h"

namespace TestPRM{

using namespace Data;
using namespace SLAM;
using namespace SLAM::Geometry;
using namespace PRM;

TestGraph::TestGraph(QObject *parent) :
    QObject(parent)
{
}

TestGraph::~TestGraph()
{

}

void TestGraph::testGraphNode(){
    //creazione nodi
    Point* p1 = new Point(1,1);
    GraphNode* gn1 = new GraphNode(p1);
    QVERIFY(gn1->x()==1);
    QVERIFY(gn1->y()==1);
    Point* p2 = new Point(3,3);
    Frontier* f = new Frontier(4,2,4,4);
    GraphNode* gn2 = new GraphNode(p2,f);
    QVERIFY(gn2->x()==3);
    QVERIFY(gn2->y()==3);
    QVERIFY(gn2->getFrontier()->centroid().x()==4);
    QVERIFY(gn2->getFrontier()->centroid().y()==3);
    QVERIFY(gn1->distance(gn2)==sqrt(8));
}

void TestGraph::testGraphEdge(){
    //creazione archi: nodi esatti
    Point* p1 = new Point(1,1);
    GraphNode* gn1 = new GraphNode(p1);
    Point* p2 = new Point(3,3);
    Frontier* f = new Frontier(4,2,4,4);
    GraphNode* gn2 = new GraphNode(p2,f);
    GraphEdge* ge = new GraphEdge(gn1,gn2);
    QVERIFY(ge->contains(gn1)==true);
    QVERIFY(ge->contains(gn2)==true);

    //creazione archi: nodo diverso con stesse coordinate
    GraphNode* gn3 = new GraphNode(p1);
    QVERIFY(ge->contains(gn3)==false);

    //test nodi Neighbours
    GraphNode* gn4 = new GraphNode(new Point(4,4));
    GraphNode* gn5 = new GraphNode(new Point(5,5));

    GraphEdge* ge2 = new GraphEdge(gn1,gn4);
    GraphEdge* ge3 = new GraphEdge(gn4,gn5);
    Graph graph= Graph();
    graph.edges.append(ge);
    graph.edges.append(ge2);
    graph.edges.append(ge3);
    QList<GraphNode*> list=graph.getNeighbours(gn1);
    QVERIFY(list.size()==2);
    QVERIFY(list.contains(gn2)==true);
    QVERIFY(list.contains(gn4)==true);
    QVERIFY(list.contains(gn5)==false);
}

void TestGraph::testNewGraph(){
    //creazione grafo
    Graph graph = Graph();
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
    Frontier* f = new Frontier(Point(10,10),Point(-10,10));
    map.addFrontier(*f);
    QVERIFY(map.frontiers().size()==1);
    QVERIFY(map.walls().size()==3);
    //primo nodo
    Point* p1 = new Point(1,1);
    graph.addNode(p1,map);
    QVERIFY(graph.nodes.size()==1);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==0);
    //nodo entro la soglia
    Point* p2 = new Point(2,2);
    graph.addNode(p2,map);
    QVERIFY(graph.nodes.size()==2);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==1);
    //nodo oltre la soglia
    Point* p3 = new Point(-9.5,-9.5);
    graph.addNode(p3,map);
    QVERIFY(graph.nodes.size()==3);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==1);
    QVERIFY(graph.frontierNodes.size()==0);
    //verifica correttezza arco
    GraphEdge* ge = graph.edges.at(0);
    QVERIFY(ge->contains(graph.nodes.at(0))==true);
    QVERIFY(ge->contains(graph.nodes.at(1))==true);
    QVERIFY(ge->contains(graph.nodes.at(2))==false);
    //punto non visibile
    Point* p4 = new Point(-10.5,-9.5);
    graph.addNode(p4,map);
    QVERIFY(graph.nodes.size()==4);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==1);
    //punto frontiera
    Point* p5 = new Point(1,2);
    graph.addNode(p5,map,f);
    QVERIFY(graph.nodes.size()==4);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==3);
    QVERIFY(graph.frontierNodes.size()==1);
    //rimozione nodo frontiera
    graph.deleteFrontierNodes(f);
    QVERIFY(graph.nodes.size()==4);
    QVERIFY(graph.frontierNodes.size()==0);
    graph.updateEdges(map);
    QVERIFY(graph.edges.size()==1);
}



}
