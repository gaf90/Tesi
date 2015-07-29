#include "astarnodeprm.h"


namespace PRM{

AStarNodePRM::AStarNodePRM()
{

}

AStarNodePRM::~AStarNodePRM(){

}

double AStarNodePRM::distance(AStarNodePRM* n){
    return node->distance(n->getNode());
}

GraphNode* AStarNodePRM::getNode(){
    return node;
}

double AStarNodePRM::getGValue(){
    return gValue;
}

double AStarNodePRM::getHValue(){
    return hValue;
}

double AStarNodePRM::getFValue(double alpha){
    //weighted
    return (alpha*hValue)+gValue;
    //static weighted
    //return ((1+alpha)*hValue)+gValue;
}

AStarNodePRM* AStarNodePRM::getParent(){
    return parent;
}

void AStarNodePRM::setGValue(double value){
    gValue=value;
}

void AStarNodePRM::setHValue(double value){
    hValue=value;
}

void AStarNodePRM::setParent(AStarNodePRM* parent){
    this->parent=parent;
}

void AStarNodePRM::setNode(GraphNode* node){
    this->node=node;
}

}
