#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include "slam/geometry/point.h"
#include "slam/geometry/frontier.h"

namespace PRM {

using namespace SLAM::Geometry;

class GraphNode
{
public:
    /**
      *Costruttore di un nodo generico
      *@param point: punto identificato dalle coordinate x,y
    */
    GraphNode(Point* point);

    /**
      *Costruttore di un nodo legato ad una frontiera
      *@param point: punto identificato dalle coordinate x,y
      *@param frontier: identifica la frontiera di appartenenza
    */
    GraphNode(Point* point, Frontier* frontier);

    /**
      *Distruttore
    */
    ~GraphNode();

    /**
      *Funzione che restituisce il valore della coordianata x
      *@return x: restitisce il valore della coordinata x
    */
    double x();

    /**
      *Funzione che restituisce il valore della coordianata y
      *@return y: restitisce il valore della coordinata y
    */
    double y();

    /**
      *Funzione che restituisce la distanza tra due nodi
      *@param node: nodo dal quale valutare la distanza
      *@return restitisce il valore della distanza
    */
    double distance(GraphNode* node);

    /**
      *Funzione che restituisce la distanza tra un nodo e un punto
      *@param p: punto dal quale valutare la distanza
      *@return restitisce il valore della distanza
    */
    double distance(Point* p);

    /**
      *Get del punto contenuto nel nodo
      *@return point: ritorna il punto che identifica il nodo
    */
    Point* getPoint();

    /**
      *Get del centro geometrico della frontiera
      *@return point: ritorna il punto che identifica il centroide
    */
    Frontier* getFrontier();

private:
    SLAM::Geometry::Point point;
    bool isFrontier;
    SLAM::Geometry::Frontier frontier;
};
}
#endif // GRAPHNODE_H
