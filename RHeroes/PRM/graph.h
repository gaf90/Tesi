#ifndef GRAPH_H
#define GRAPH_H

#include "slam/geometry/point.h"
#include "slam/map.h"
#include "shared/config.h"
#include "slam/utilities.h"
#include "graphnode.h"
#include "graphedge.h"
#include <QList>
#include "testPRM/testgraph.h"
#include "testPRM/testprmalgorithm.h"
#include "testPRM/testastarprm.h"

namespace PRM {

using namespace SLAM;

class Graph
{
    friend class TestPRM::TestGraph;
    friend class TestPRM::TestAStarPRM;
    friend class TestPRM::TestPRMAlgorithm;
public:
    /**
      *Costruttore grafo
    */
    Graph();

    /**
      *Distruttore grafo
    */
    ~Graph();

    /**
      *Crea un nuovo nodo che identifica il punto passato come parametro
      *@param point: punto identificato dalle coordinate x,y
      *@param map: la mappa conosciuta dall'ambiente
    */
    GraphNode* addNode(Point* point, Map map);

    /**
      *Crea un nuovo nodo che identifica il punto passato come parametro situato nelle vicinanze di una frontiera
      *@param point: punto identificato dalle coordinate x,y
      *@param map: la mappa conosciuta dall'ambiente
      *@param centroid: identifica il centro geometrico della frontiera
    */
    GraphNode* addNode(Point* point, Map map, Frontier* frontier);

    /**
      *Elimina i nodi associati ad una frontiera identificata attraverso il suo centroide
      *@param centroid: identifica il centro geometrico della frontiera
    */
    void deleteFrontierNodes(Frontier* frontier);

    void plot(QTextStream* stream);

    GraphNode* nearestNode(Point* point);

    QList<GraphNode*> getNeighbours(GraphNode* node);

    GraphNode* nearestToDestination(Point* point);

    void updateEdges(Map map);

    QList<GraphNode*> getNodes();

    QList<GraphNode*> getFrontierNodes();

    QList<GraphEdge*> getEdges();


private:
    QList<GraphNode*> nodes;
    QList<GraphNode*> frontierNodes;
    QList<GraphEdge*> edges;

};
}
#endif // GRAPH_H
