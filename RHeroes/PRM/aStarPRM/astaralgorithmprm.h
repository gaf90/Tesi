#ifndef ASTARALGORITHMPRM_H
#define ASTARALGORITHMPRM_H
#include "PRM/graph.h"
#include "PRM/graphnode.h"
#include "astarnodeprm.h"

namespace PRM{

typedef QList<Point*> PRMPath;

class AStarAlgorithmPRM
{

public:
    AStarAlgorithmPRM(Map m, Graph g);

    /**
      *Distruttore AStarAlgorithmPRM
    */
    ~AStarAlgorithmPRM();
    PRMPath getPath(double alpha, Point* destination);
    QList<GraphNode*> findChildren(AStarNodePRM* node);
    AStarNodePRM* chooseNextNode(double alpha);
    QList<Point*> calculatePath(AStarNodePRM* node);
    AStarNodePRM* inClosedSet(GraphNode* node);
    AStarNodePRM* inOpenSet(GraphNode* node);

private:
    Map map;
    AStarNodePRM* start;
    AStarNodePRM* goal;
    QList<AStarNodePRM*> *closedSet;
    QList<AStarNodePRM*> *openSet;
    GraphNode* startNode;
    GraphNode* goalNode;
    Graph* graph;

};

}
#endif // ASTARALGORITHMPRM_H
