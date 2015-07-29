#include "graphedge.h"

namespace PRM {
using namespace SLAM::Geometry;

GraphEdge::GraphEdge(GraphNode* a, GraphNode* b):
    a(a), b(b){}
bool GraphEdge::contains(GraphNode* node){
    return (node==a) || (node==b);
}


GraphNode* GraphEdge::getA(){
    return a;
}
GraphNode* GraphEdge::getB(){
    return b;
}

double GraphEdge::length(){
    return a->distance(b);
}

}
