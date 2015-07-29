#include "rrtalgorithm.h"
#include "shared/utilities.h"
#include "slam/geometry/linesegment.h"
#include "pathPlanner/pathplannerutils.h"
#include <cmath>

namespace PathPlanner{

#if defined(Q_OS_WIN32)
#   define isnan _isnan
#elif defined(Q_OS_MACX)
#   define isnan std::isnan
#endif

using namespace SLAM;
using namespace Geometry;

int maxIter =0;

RRTAlgorithm::RRTAlgorithm(Map *map, RRTNode *start, RRTNode *goal, double constraint) :
    map(map),
    start(start),
    goal(goal),
    constraint(constraint),
    generated(new QSet<RRTNode *>())
{
    generated->insert(start);
    if(constraint!=-1){
        this->constraint = fromDegreeToRadiants(constraint);
    }
}

RRTAlgorithm::~RRTAlgorithm()
{
//    QSetIterator<RRTNode *> it(*generated);
//    while(it.hasNext()){
//        RRTNode *toRem = it.next();
//        generated->remove(toRem);
//        if(toRem != start)
//            delete toRem;
//    }
//    delete generated;
    //delete goal;
}

QStack<Data::Action *> *RRTAlgorithm::doAlgorithm(int delta)
{
    if(constraint == -1){
        unconstrainedAlgorithm(delta);
    } else {
        constrainedAlgorithm(delta);
    }

    ldbg <<"Sono uscito"<<endl;
    QStack<Data::Action *> *toRet;
    if (maxIter>2499)
        return toRet = new QStack<Data::Action *>();
    else{
         toRet = computeLowLevelAction();

    return toRet;
    }
}

QStack<Data::Action *> * RRTAlgorithm::computeLowLevelAction()
{
    QStack<Data::Action *> *stack = new QStack<Data::Action *>();
    RRTNode *tmpNode = goal;
    while(tmpNode->getParent() != NULL){
        RRTNode *parent = tmpNode->getParent();

        stack->push(new Data::Action(Data::Action::Translation , EuclideanDistance(*tmpNode, *parent)));
        stack->push(new Data::Action(Data::Action::Rotation , tmpNode->getSteeringAngle()));
        tmpNode = parent;
    }
    return stack;
}

RRTNode * RRTAlgorithm::randomNode()
{
    int random = rand() % 100;
    if(random < 10){
        return goal;
    }

    RRTNode * toRet = NULL;
    double x = 0, y = 0;
    bool valid = false;

    while(!valid){

        x = (((double)rand()/(double)(RAND_MAX)) * ((map->getMaxX()-map->getMinX())*100))-fabs(map->getMinX());
        x = x/100;
        y = (((double)rand()/(double)(RAND_MAX)) * ((map->getMaxY()-map->getMinY())*100))-fabs(map->getMinY());
        y = y/100;
        toRet = new RRTNode(x, y);
        valid = validPoint(x, y, map) && !generated->contains(toRet);
    }

    return toRet;
}


RRTNode * RRTAlgorithm::nearNode(RRTNode *close, RRTNode *rand, int delta)
{
    RRTNode *toRet = NULL;
    double distStart = EuclideanDistance(*close, *rand);

    //check if the sample is already at a distance that is less than delta.
    if(distStart < delta){
        return rand;
    }
    //search for the straight line from close to rand
    double dy = rand->getY() - close->getY();
    double dx = rand->getX() - close->getX();
    double x0 = close->getX(), y0 = close->getY();
    double m = dy / dx;
    double q = close->getY()-m*close->getX();

    //Parameters of the circumference.
    double a = 1+pow(m,2);
    double b = -2*x0+2*m*q-2*m*y0;
    double c = -(pow(delta, 2))+(pow(x0, 2))+(pow(q, 2))+(pow(y0, 2))-2*y0*q;

    //I search for the two points that are at distance delta from the close node.
    double xTent1 = (-b + sqrt(pow(b, 2) - 4*a*c)) / (2*a);
    double yTent1 = m*xTent1+q;
    double xTent2 = (-b - sqrt(pow(b, 2) - 4*a*c)) / (2*a);
    double yTent2 = m*xTent2+q;

    //I check the validity of the nodes
    RRTNode *tentNode1 = new RRTNode(xTent1, yTent1);
    if(!isnan(xTent1) && !isnan(yTent1))
        tentNode1->setValid(true);

    RRTNode *tentNode2 = new RRTNode(xTent2, yTent2);
    if(!isnan(xTent2) && !isnan(yTent2))
        tentNode2->setValid(true);

    //I choose the node that is closest to the random node.
    toRet = tentNode1;
    if(EuclideanDistance(*rand, *tentNode2) < EuclideanDistance(*rand, *toRet)){
        toRet = tentNode2;
    }

    return toRet;
}

RRTNode * RRTAlgorithm::closestNode(RRTNode *graph, RRTNode *rand)
{
    RRTNode *bestNode = graph;
    QSetIterator<RRTNode *> it(*generated);
    double distance = EuclideanDistance(*bestNode, *rand);
    while(it.hasNext()){
        RRTNode *nextNode = it.next();
        double tmpDist = EuclideanDistance(*nextNode, *rand);
        if(tmpDist < distance){
            distance=tmpDist;
            bestNode=nextNode;
        }
    }

    return bestNode;
}

RRTNearClosePair *RRTAlgorithm::constrainedClosestNode(RRTNode *graph, RRTNode *rand, int delta)
{
    Q_UNUSED(graph) Q_UNUSED(rand) Q_UNUSED(delta)
    return NULL;
}

void RRTAlgorithm::unconstrainedAlgorithm(int delta)
{
    bool end = false, firstIter = true;
    while(!end && maxIter<2500){
        maxIter++;
        RRTNode *rand = NULL;
        if(firstIter){
            rand = goal;
            firstIter = false;
        } else {
            rand = randomNode();
        }
        RRTNode *close = closestNode(start, rand);
        RRTNode *near = nearNode(close, rand, delta);

        if(inFreeSpace(LineSegment(close->getX(), close->getY(), near->getX(), near->getY()), map)){
            computeOrientation(near, close, constraint);
            close->addReachable(near);
            near->setParent(close);
            generated->insert(near);
            ldbg<<"RRT"<<endl;
            if(near == goal)
                    end = true;
        }

        if(!(rand == goal) && !generated->contains(rand)){
            delete rand;
        }

    }
}

void RRTAlgorithm::constrainedAlgorithm(int delta)
{
    bool end = false, firstIter = true;
    while(!end){
        RRTNode *rand = NULL;
        if(firstIter){
            rand = goal;
            firstIter = false;
        } else {
            rand = randomNode();
        }
        RRTNearClosePair *pair = constrainedClosestNode(start, rand, delta);

        //Check near node validity: a consistent point && not in an obstacole


            pair->getClose()->addReachable(pair->getNear());
            pair->getNear()->setParent(pair->getClose());

            if(pair->getNear() == goal)
                end = true;

        if(!(rand == goal) && !generated->contains(rand)){
            delete rand;
        }
    }
}
}
