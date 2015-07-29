#include "graphnode.h"

namespace PRM {

using namespace SLAM::Geometry;


GraphNode::GraphNode(SLAM::Geometry::Point* point):
    point(Point(point->x(),point->y())), isFrontier(false), frontier(){}

GraphNode::GraphNode(SLAM::Geometry::Point* point, SLAM::Geometry::Frontier* f):
    point(Point(point->x(),point->y())), isFrontier(true), frontier(Frontier(f->x1(),f->y1(),f->x2(),f->y2())){}

/**
  *Distruttore grafo
*/
GraphNode::~GraphNode(){
}

double GraphNode::x(){
    return point.x();
}

double GraphNode::y(){
    return point.y();
}

double GraphNode::distance(GraphNode* node){
    double dx = point.x()- node->x();
    double dy = point.y()- node->y();
    return std::sqrt((dx*dx)+(dy*dy));
}

double GraphNode::distance(Point* p){
    double dx = point.x()- p->x();
    double dy = point.y()- p->y();
    return std::sqrt((dx*dx)+(dy*dy));
}

Point* GraphNode::getPoint(){
    return &point;
}

Frontier* GraphNode::getFrontier(){
    return &frontier;
}
}
