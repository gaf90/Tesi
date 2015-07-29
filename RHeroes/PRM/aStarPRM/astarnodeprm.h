#ifndef ASTARNODEPRM_H
#define ASTARNODEPRM_H
#include <QList>
#include "PRM/graphnode.h"


namespace PRM{



class AStarNodePRM
{

public:
    AStarNodePRM();

    /**
      *Distruttore AStarNodePRM
    */
    ~AStarNodePRM();
    double distance(AStarNodePRM* n);
    GraphNode* getNode();
    double getGValue();
    double getHValue();
    double getFValue(double alpha);
    AStarNodePRM* getParent();
    void setGValue(double value);
    void setHValue(double value);
    void setParent(AStarNodePRM* parent);
    void setNode(GraphNode* node);

private:
    double gValue;
    double hValue;
    GraphNode* node;
    AStarNodePRM* parent;
};

}
#endif // ASTARNODEPRM_H
