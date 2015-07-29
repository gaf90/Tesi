#include "graph.h"
#include <QDebug>
#include <QList>

namespace PRM {

using namespace SLAM;

Graph::Graph():
    nodes(QList<GraphNode *>()), frontierNodes(QList<GraphNode *>()), edges(QList<GraphEdge *>()){}

Graph::~Graph(){
    //    foreach(GraphNode* n, nodes){
    //        delete n;
    //        nodes.removeOne(n);
    //    }
    //    foreach(GraphEdge* e, edges){
    //        delete e;
    //        edges.removeOne(e);
    //    }
    //    foreach(GraphNode* f, frontierNodes){
    //        delete f;
    //        frontierNodes.removeOne(f);
    //    }
}

GraphNode* Graph::addNode(Point* point, Map map){
    GraphNode* node = new GraphNode(point);
    //    foreach(GraphNode* n, nodes+frontierNodes){
    //        if(node->distance(n)<Config::PRM::edgeThreshold){// && node.distance(n)>0
    //            if(map.isReachable(Point(node->x(),node->y()),Point(n->x(),n->y()),0)==True){
    //                GraphEdge* edge= new GraphEdge(node, n);
    //                edges.append(edge);
    //            }
    //        }
    //    }
    nodes.append(node);
    return node;
}

GraphNode* Graph::addNode(Point* point, Map map, Frontier* f){
    GraphNode* node = new GraphNode(point,f);
    //    foreach(GraphNode* n, nodes+frontierNodes){
    //        if(node->distance(n)<Config::PRM::edgeThreshold){// && node.distance(n)>0
    //            if(map.isReachable(Point(node->x(),node->y()),Point(n->x(),n->y()),0)==True){
    //                GraphEdge* edge= new GraphEdge(node, n);
    //                edges.append(edge);
    //            }
    //        }
    //    }
    frontierNodes.append(node);
    return node;
}

void Graph::deleteFrontierNodes(Frontier* frontier){
    foreach(GraphNode* n, frontierNodes){
        if(n->getFrontier()->centroid()==frontier->centroid()){
            frontierNodes.removeOne(n);
            foreach(GraphEdge* e, edges){
                if(e->contains(n)){
                    edges.removeOne(e);
                }
            }
        }
    }
}

void Graph::plot(QTextStream* stream){
    //stampa archi
    //ldbg << "PRM Algorithm: plot graph" << endl;
    if(!edges.isEmpty()){
        QString xEdgeIni= "", yEdgeIni="", xEdgeFin="", yEdgeFin="";
        foreach(GraphEdge* e, edges){
            xEdgeIni.append(" ").append(QString::number(e->getA()->x(),'f',4));
            yEdgeIni.append(" ").append(QString::number(e->getA()->y(),'f',4));
            xEdgeFin.append(" ").append(QString::number(e->getB()->x(),'f',4));
            yEdgeFin.append(" ").append(QString::number(e->getB()->y(),'f',4));
        }
        (*stream)<<"xEdges=["<<xEdgeIni<<"; "<< xEdgeFin <<"];"<<endl;
        (*stream)<<"yEdges=["<<yEdgeIni<<"; "<< yEdgeFin <<"];"<<endl;
    }
    else{
        (*stream)<<"xEdges=[0;0];"<<endl;
        (*stream)<<"yEdges=[0;0];"<<endl;
    }
    (*stream)<<"edges = plot(xEdges, yEdges, '-.g');"<<endl;
    //stampa punti nodi
    QString xNodes= "", yNodes="";
    foreach(GraphNode* n, nodes){
        xNodes.append(" ").append(QString::number(n->getPoint()->x(),'f',4));
        yNodes.append(" ").append(QString::number(n->getPoint()->y(),'f',4));
    }
    (*stream)<<"xNodes=["<<xNodes<<"];"<<endl;
    (*stream)<<"yNodes=["<<yNodes<<"];"<<endl;
    (*stream)<<"nodes = plot(xNodes, yNodes, 'g.');"<<endl;

    //stampa nodi frontiera
    if(!frontierNodes.empty()){
        QString xNodesFrontiers= "", yNodesFrontiers="";
        foreach(GraphNode* nf, frontierNodes){
            xNodesFrontiers.append(" ").append(QString::number(nf->getPoint()->x(),'f',4));
            yNodesFrontiers.append(" ").append(QString::number(nf->getPoint()->y(),'f',4));
        }
        (*stream)<<"xNodesFrontiers=["<<xNodesFrontiers<<"];"<<endl;
        (*stream)<<"yNodesFrontiers=["<<yNodesFrontiers<<"];"<<endl;
        (*stream)<<"frontiersNodes = plot(xNodesFrontiers, yNodesFrontiers, 'r.');"<<endl;
    }
}

GraphNode* Graph::nearestNode(Point* point){
    double distance=INFINITY;
    GraphNode* node=NULL;
    foreach(GraphNode* n, nodes){
        if(n->distance(point)<distance){
            distance=n->distance(point);
            node= n;
        }
    }
    return node;
}

//usata per A*
QList<GraphNode*> Graph::getNeighbours(GraphNode* node){
    QList<GraphNode*> list;
    foreach(GraphEdge* e, edges){
        if(e->contains(node)){
            GraphNode* neighbour = NULL;
            if(*(node->getPoint())==*(e->getA()->getPoint())){
                neighbour=e->getB();
            }
            else{
                neighbour=e->getA();
            }
            list.append(neighbour);
        }
    }
    return list;
}

//aggiornamento archi
void Graph::updateEdges(Map map){
    edges.clear();
    //int size= nodes.size()+ frontierNodes.size();
    QList<GraphNode*> tempList = nodes+frontierNodes;
    int size= tempList.size();
    for(int i=0; i<size; i++){
        for(int j=i+1; j<size; j++){
            GraphNode* n1= tempList.at(i);
            GraphNode* n2= tempList.at(j);
            if(n1->distance(n2)<Config::PRM::edgeThreshold){
               if(map.isReachable(*n1->getPoint(),*n2->getPoint(),0)==True){
                    if(map.isReachable(*n1->getPoint(),*n2->getPoint(),Config::PRM::movementRadius)!=False){
                        GraphEdge* edge=new GraphEdge(n1,n2);
                        edges.append(edge);
                    }
               }
            }
        }
    }
}

GraphNode* Graph::nearestToDestination(Point* point){
    double distance=INFINITY;
    GraphNode* node=NULL;
    if(!frontierNodes.empty()){
        foreach(GraphNode* n, frontierNodes){
            if(n->distance(point)<distance){
                distance=n->distance(point);
                node= n;
            }
        }
    }
    if(distance>Config::PRM::movementRadius){
        foreach(GraphNode* n, nodes){
            if(n->distance(point)<distance){
                distance=n->distance(point);
                node= n;
            }
        }
    }    
    if(distance>Config::PRM::movementRadius*2){
        //ldbg<<"Can not find a node closes to the target. Distance: "<<distance<<endl;
        return NULL;
    }
    return node;
}

QList<GraphNode*> Graph::getNodes(){
    return nodes;
}

QList<GraphNode*> Graph::getFrontierNodes(){
    return frontierNodes;
}

QList<GraphEdge*> Graph::getEdges(){
    return edges;
}

}
