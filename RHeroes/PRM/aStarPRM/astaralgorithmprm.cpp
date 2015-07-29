#include "astaralgorithmprm.h"
#include <QElapsedTimer>

namespace PRM{

AStarAlgorithmPRM::AStarAlgorithmPRM(Map m, Graph g):
    map(m), start(NULL), goal(NULL), closedSet(new QList<AStarNodePRM*>()),
    openSet(new QList<AStarNodePRM*>())
{
    QElapsedTimer timer;
    timer.start();
    graph=new Graph();
    foreach(GraphNode* n, g.getNodes()){
        graph->addNode(n->getPoint(),map);
    }
    foreach(GraphNode* n, g.getFrontierNodes()){
        graph->addNode(n->getPoint(),map,n->getFrontier());
    }
    const PathNode* robot=map.lastRobotPose(Config::robotID);
    Point* startPoint= new Point(robot->x(), robot->y());
    startNode=graph->addNode(startPoint,map);
    start= new AStarNodePRM();
    start->setGValue(0);
    start->setParent(NULL);
    start->setNode(startNode);
    graph->updateEdges(map);
}

AStarAlgorithmPRM::~AStarAlgorithmPRM(){
    delete start;
    delete goal;
    for(int i =openSet->size()-1;i>=0;i--){
        delete openSet->at(i);
        openSet->removeAt(i);
    }
    delete openSet;
    for(int i =closedSet->size()-1;i>=0;i--){
        delete closedSet->at(i);
        closedSet->removeAt(i);
    }
    delete closedSet;
}

PRMPath AStarAlgorithmPRM::getPath(double alpha, Point* destination){
    QElapsedTimer timer;
    timer.start();
    int expandedNodes=0;
    goalNode= graph->nearestToDestination(destination);
    if(goalNode==NULL){
        ldbg << "PRM A*: path not found (alpha: "<<alpha<<") expanded nodes: "<<expandedNodes<< endl;
        return QList<Point*>();
    }
    start->setHValue(start->getNode()->getPoint()->distance(*destination));
    goal= new AStarNodePRM();
    goal->setHValue(0);
    goal->setParent(NULL);
    goal->setNode(goalNode);
    openSet->clear();
    openSet->append(start);
    closedSet->clear();
    while(!openSet->isEmpty()){
        AStarNodePRM* current = chooseNextNode(alpha);
        expandedNodes++;
        //ldbg<<"current node: "<<current->getNode()->getPoint()<<endl;
        if(current->getNode()==goal->getNode()){
            PRMPath path = calculatePath(current);
            delete goal;
            ldbg << "PRM A*: path creation (alpha: "<<alpha<<") expanded nodes: "<<expandedNodes<<" , time: " << timer.elapsed()<<"ms"<< endl;
            return path;
        }
        openSet->removeOne(current);
        closedSet->append(current);
        foreach(GraphNode* n, findChildren(current)){
            //ldbg<<"children: "<<n->getPoint()<<endl;
            if(inClosedSet(n)!=NULL){
                //                ldbg<<"child in closed set"<<endl;
                continue;
            }
            else{
                AStarNodePRM* neighbour = inOpenSet(n);
                if(neighbour!=NULL){
                    //                    ldbg<<"child in open set"<<endl;
                    double gValue = current->getGValue()+ current->distance(neighbour);
                    //double gValue = current->getGValue()+ 1;
                    if(neighbour->getGValue()> gValue){
                        neighbour->setParent(current);
                        neighbour->setGValue(gValue);
                    }
                }
                else{
                    AStarNodePRM* child = new AStarNodePRM();
                    //                    ldbg<<"new child created"<<endl;
                    child->setParent(current);
                    child->setNode(n);
                    double distance = current->getGValue()+ current->distance(child);
                    child->setGValue(distance);
                    child->setHValue(child->getNode()->distance(goalNode->getPoint()));
                    openSet->append(child);
                }
            }
        }
    }
    delete goal;
    ldbg << "PRM A*: path not found (alpha: "<<alpha<<") expanded nodes: "<<expandedNodes<< endl;
    return QList<Point*>();
}

QList<GraphNode*> AStarAlgorithmPRM::findChildren(AStarNodePRM* node){
    QList<GraphNode*> neighbours= graph->getNeighbours(node->getNode());
    //ldbg<<"found "<<neighbours.size()<<" neighbours"<<endl;
    return neighbours;
    //    foreach(GraphNode* n, neighbours){
    //        AStarNodePRM* child = new AStarNodePRM();
    //        child->setParent(node);
    //        child->setNode(n);
    //        double distance = node->getGValue()+ node->distance(child);
    //        child->setGValue(distance);
    //        child->setHValue(child->getNode()->distance(goalNode->getPoint()));
    //        node->addChild(child);
    //    }
}

AStarNodePRM* AStarAlgorithmPRM::chooseNextNode(double alpha){
    double bestValue= INFINITY;
    AStarNodePRM* bestNode = NULL;
    foreach(AStarNodePRM* n, *openSet){
        if(n->getFValue(alpha)<bestValue){
            bestValue=n->getFValue(alpha);
            bestNode= n;
        }
    }
    return bestNode;
}


QList<Point*> AStarAlgorithmPRM::calculatePath(AStarNodePRM* node){
    QList<Point*> path = QList<Point*>();
    do{
        Point* point = new Point(node->getNode()->getPoint()->x(),node->getNode()->getPoint()->y());
        path.prepend(point);
        node= node->getParent();
    }while(node!=NULL);
    return path;
}


AStarNodePRM* AStarAlgorithmPRM::inClosedSet(GraphNode* node){
    foreach(AStarNodePRM* n, *closedSet){
        if(n->getNode()==node){
            return n;
        }
    }
    return NULL;
}

AStarNodePRM* AStarAlgorithmPRM::inOpenSet(GraphNode* node){
    foreach(AStarNodePRM* n, *openSet){
        if(n->getNode()==node){
            return n;
        }
    }
    return NULL;
}

}
