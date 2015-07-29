#ifndef GRAPHEDGE_H
#define GRAPHEDGE_H

#include "slam/geometry/point.h"
#include "graphnode.h"

namespace PRM {
using namespace SLAM::Geometry;

class GraphEdge
{
public:
    /**
      *Costruttore
      *@param a: nodo ad un'estremità dell'arco
      *@param b: nodo ad un'estremità dell'arco
    */
    GraphEdge(GraphNode* a, GraphNode* b);


    /**
      *Verifica se il nodo appartiene all'arco
      *@param node: punto identificato dalle coordinate x,y
      *@return true se il nodo appartiene all'arco
    */
    bool contains(GraphNode* node);
    GraphNode* getA();
    GraphNode* getB();

    double length();

private:
    GraphNode* a;
    GraphNode* b;

};
}
#endif // GRAPHEDGE_H
