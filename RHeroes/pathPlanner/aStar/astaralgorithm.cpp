#include "astaralgorithm.h"
#include "astarnodecomparator.h"
#include "exploration/explorationconstants.h"
#include "pathPlanner/pathplannerutils.h"
#include "shared/random.h"
#include <QTime>

namespace PathPlanner{

using namespace SLAM;
using namespace SLAM::Geometry;

AStarAlgorithm::AStarAlgorithm(uint identifier,const SLAM::Map *slamMap, double xGoal, double yGoal) :
    slamMap(slamMap), startNode(NULL), goalNode(NULL), robotPose(NULL), goalPose(NULL), shouldEnd(false),
    closedSet(new QList<AStarNode*>()), openSet(new QList<AStarNode*>())
{
    robotPose = slamMap->lastRobotPose(identifier);

    startNode = new AStarNode();
    startNode->setGValue(0);
    startNode->setHValue(Point(robotPose->getX(), robotPose->getY()).distance(Point(xGoal, yGoal)));
    startNode->setParent(NULL);
    PathNode *robotPoseNew = new PathNode(*robotPose);
    startNode->setPose(robotPoseNew);
    startNode->setOwnPose(true);

    goalNode = new AStarNode();
    goalNode->setHValue(0);
    goalNode->setParent(NULL);
    goalPose = new PathNode(0.0, Data::Pose(xGoal, yGoal, 0.0));
    goalNode->setPose(goalPose);

    openSet->append(startNode);
}

AStarAlgorithm::~AStarAlgorithm()
{
    delete goalPose;
    delete goalNode;
    delete startNode;

    for(int i=openSet->length()-1; i>=0; i--){
        delete openSet->at(i);
        openSet->removeAt(i);
    }
    delete openSet;
    delete closedSet;


}

QStack<AbstractAction *> * AStarAlgorithm::doAlgorithm()
{
    QStack<AbstractAction *> *toRet = NULL;
    while(!openSet->isEmpty()){ //untile the open set is empty
        //choose a node to expand from the open set (the one with lower f)
        AStarNode* toExpand = chooseANodeToExpand();
        //then, check if i have already explored that node.
        if(!closedSet->contains(toExpand)){
            //printPath(toExpand);
            //If i do not have explored the node, i have something to do:
            if(toExpand == goalNode){
                //The node is the goal? YES.
                //Compute and return the actions I should take to reach it
                toRet = computePathToTheGoal(toExpand);
            } else {
                //The node is the goal? NO.
                //Move it from the openSet to the closedSet
                openSet->removeFirst();
                closedSet->append(toExpand);

                //search for frontiers. I know that the goal must be a frontier
                //(or, in future, a victim.. maybe I need a parameter to set in the constructor)
                AStarNode *nodeFound = frontierFound(toExpand);
                if(nodeFound != NULL){
                    //I have found a node that represent the goal among my frontiers (or victims...)
                    //ldbg << "=== PLAN PRINT ===" << endl;
                    toRet = computePathToTheGoal(nodeFound);
                    //ldbg << "=== END PLAN PRINT ===" << endl;
                } else if(shouldEnd){
                    ldbg << "$$$ No SafePoint Found." << endl;
                    return NULL; //Algorithm Fails
                } else {
                    //Among the current node frontiers, there is not the goal
                    //Due to the fact that PathNode has only two neighbours, i must check them singularly
                    bool nextNull = true, prevNull = true;
                    if(toExpand->getPose()->next() != NULL){
                        nextNull = false;
                        //Next node is not null. I create a new AStarNode that represents it
                        //and i add it to the openSet.
                        createNewNode(toExpand->getPose()->next(), toExpand);
                    }
                    if(toExpand->getPose()->previous() != NULL){
                        prevNull = false;
                        //Prev node is not null. I create a new AStarNode that represents it
                        //and i add it to the openSet.
                        createNewNode(toExpand->getPose()->previous(), toExpand);

                    }

                    if(nextNull && prevNull)
                        return NULL; //Algorithm fails!
                }

            }
        }

    }
    return toRet;
}

AStarNode* AStarAlgorithm::chooseANodeToExpand()
{
    //To choose the next node to pick, I sort the nodes by their f value.
    qSort(openSet->begin(), openSet->end(), AStarNodeComparator());
    //I take the first node as the cheapest one.
    return openSet->first();
}

AStarNode *AStarAlgorithm::frontierFound(AStarNode *node)
{
    //Search among my frontiers if there is the goal.
    const QList<SLAM::Geometry::Frontier *> frontiers = node->getPose()->visibleFrontiers();
    foreach(Frontier *f, frontiers){
        //For each frontier, check if it is quite next to the goal.
        //Due to the stochastic nature of the map, it is very difficult that
        //the <x, y> values of the frontiers remain the same.
        if(f->centroid().distance(Point(goalPose->getX(), goalPose->getY())) < MAX_FRONT_RADIUS){
            //I found the goal frontier!
            Point *safePoint = computeSafePoint(f, node->getPose());
            if(safePoint == NULL){
                shouldEnd = true;
                return NULL;
            }
            AStarNode* toRet = new AStarNode();
            toRet->setPose(new PathNode(0.0, Data::Pose(safePoint->x(), safePoint->y(), 0.0)));
            toRet->setGValue(toRet->distance(node)+node->getGValue());
            toRet->setHValue(toRet->distance(goalNode));
            toRet->setParent(node);
            toRet->setOwnPose(true); //Because i create an artificial pose, the node must destroy it when the Algorithm destructor will be called
            node->addChild(toRet);
            delete safePoint; //I do not need the safe point anymore
            return toRet;
        }

    }
    //I finished to check among all the known frontiers and i have not found the goal...
    return NULL;

}

void AStarAlgorithm::createNewNode(const SLAM::PathNode *pathNode, AStarNode *toExpand){
    //Given a path node, i need to create an AStarNode
    AStarNode *newNode = new AStarNode();
    PathNode *newPathNodeToSet = new PathNode(*pathNode);
    newNode->setPose(newPathNodeToSet); //The pose of the AStarnode must be the pathNode's pose.
    newNode->setOwnPose(true);
    newNode->setGValue(toExpand->getGValue()+toExpand->distance(newNode));
    newNode->setHValue(newNode->distance(goalNode));

    bool closedSetContains = false;
    foreach(AStarNode *node, *closedSet){
        Point nodePoint(node->getPose()->getX(), node->getPose()->getY());
        Point newNodePoint(newNode->getPose()->getX(), newNode->getPose()->getY());
        double distance = nodePoint.distance(newNodePoint);
        //ldbg << "Distance between new node and node in the closed set is " << distance << endl;
        if(distance == 0.0 && node->getPose()->getTheta() == newNode->getPose()->getTheta()){
            closedSetContains = true;
            break;
        }
    }

    if(closedSetContains){
        //If the closed set contains the node yet, i do not need to create a new node
        //I have already explored this node before.
        ldbg << "the closed set already contains the node" << endl;
        delete newNode;
        return;
    }

    bool openSetContains = false;
    foreach(AStarNode *node, *openSet){
        Point nodePoint(node->getPose()->getX(), node->getPose()->getY());
        Point newNodePoint(newNode->getPose()->getX(), newNode->getPose()->getY());
        double distance = nodePoint.distance(newNodePoint);
        //ldbg << "Distance between new node and node in the OPEN set is " << distance << endl;

        if(distance < ROBOT_DIAG && node->getPose()->getTheta() == newNode->getPose()->getTheta()){
            openSetContains = true;
            break;
        }
        }

    if(!openSetContains){
        //If the open set does not contain the node,
        //I set the parenthood and childhood of the new node
        newNode->setParent(toExpand);
        toExpand->addChild(newNode);
        //and add the new node to the openset to explore it in the future.
        openSet->append(newNode);
        ldbg << "new node into the openSet" << endl;
    } else {
        ldbg << "***Open set already contains the computed node" << endl;
        //the open set contains the new node.
        AStarNode *nodeInOpenSet = openSet->at(openSet->indexOf(newNode));
        if(nodeInOpenSet->getGValue()>newNode->getGValue()){
            //I compare their costs to check if the new one is better than the second one.
            //If yes, i change its past values for G, and parent
            nodeInOpenSet->setGValue(newNode->getGValue());
            nodeInOpenSet->setParent(toExpand);
            //I mark the node in the open set as a child for the node to expand.
            toExpand->addChild(nodeInOpenSet);
        }

        //Because i won't use the new node, I can safely destroy it.
        delete newNode;
    }
}

QStack<AbstractAction *> * AStarAlgorithm::computePathToTheGoal(AStarNode *goalFound)
{
    //I need the sequence of poses i will follow to correctly build the actions.
    QStack<AStarNode *> path;
    path.push(goalFound); //add the goal
    AStarNode* nodeToProcess = goalFound->getParent();
    while(nodeToProcess != startNode){
        path.push(nodeToProcess);
        nodeToProcess = nodeToProcess->getParent();
    }
    //Popping the node from the path's stack, i can compute the actions I need.
    //Now i need to compute:
    // - a rotation
    // - the theta of the new pose
    // - a transaltion
    //then, i have to reverse the stack in order to pop the things in the correct order.
    nodeToProcess = path.pop();
    AStarNode *actualNode = startNode;
    QStack<Data::Action *> tempActions;
    do{
        const PathNode *startPose = actualNode->getPose();
        const PathNode *toReachPose = nodeToProcess->getPose();

        double angle2 = computeRotationFromPoses(*startPose, *toReachPose);
        Data::Action *rotAction = NULL;
        if(angle2 != 0.0){
            rotAction = new Data::Action();
            rotAction->setType(Data::Action::Rotation);
            rotAction->setValue(fromRadiantToDegree(angle2));
            tempActions.push(rotAction);
        }
        //Set the theta value of the pose.
        PathNode *newPathNodePose = new PathNode(nodeToProcess->getPose()->getTimestamp(),
                                                             Data::Pose(nodeToProcess->getPose()->getX(),
                                                                        nodeToProcess->getPose()->getY(),
                                                                        startPose->getTheta()+angle2));
        const PathNode *prevPathNode = nodeToProcess->getPose();
        nodeToProcess->setPose(newPathNodePose);
        nodeToProcess->setOwnPose(true);
        if(nodeToProcess->getOwnPose()){
            delete prevPathNode;
        }
        toReachPose = newPathNodePose;
        //compute the translation
        double dx2 = pow(toReachPose->getX()-startPose->getX(), 2);
        double dy2 = pow(toReachPose->getY()-startPose->getY(), 2);
        double trasl = sqrt(dy2+dx2);
        Data::Action *traAction = NULL;
        if(fabs(trasl) > 0.1){
            traAction = new Data::Action();
            traAction->setType(Data::Action::Translation);
            traAction->setValue(trasl);
            tempActions.push(traAction);
        }


        //printPlan(actualNode, nodeToProcess, rotAction, traAction);


        //nodes updates
        actualNode = nodeToProcess;
        if(!path.isEmpty())
            nodeToProcess = path.pop();
    } while(!path.isEmpty());
    //I add the last rotation to let the robot to face the frontier.

    double lastAngle = computeRotationFromPoses( *(nodeToProcess->getPose()) , *goalPose);
    if(lastAngle != 0.0){
        Data::Action *lastAction = new Data::Action();
        lastAction->setType(Data::Action::Rotation);
        lastAction->setValue(fromRadiantToDegree(lastAngle));
        tempActions.push(lastAction);
    }

    //At last, i reverse the tempActions stack into the stack i must return.
    QStack<AbstractAction *> *toRet = new QStack<AbstractAction *>();
    while(!tempActions.isEmpty()){
        toRet->push(tempActions.pop());
    }
    return toRet;
}

SLAM::Geometry::Point* AStarAlgorithm::computeSafePoint(SLAM::Geometry::Frontier *f, const SLAM::PathNode *node)
{

    double radius = SAFETY_RADIUS;
    Point *p = NULL;
    double sigmaX = 0; //Shared::Random::uniform(0, SAFETY_RADIUS/2);
    double sigmaY = 0; //Shared::Random::uniform(0, SAFETY_RADIUS/2);
    //for(int i=0; i<MAX_ITER; i++){
        double anglePlusHalfPi = f->angle()+M_PI/2;
        if(p != NULL)
            delete p;
        p = new Point(f->centroid().x()+radius*cos(anglePlusHalfPi)+sigmaX, f->centroid().y()+radius*sin(anglePlusHalfPi)+sigmaY);
        if(!foundIntersection(*p, node))
            return p;

        double angleMinusHalfPi = f->angle()-M_PI/2;
        if(p!=NULL)
            delete p;
        p = new Point(f->centroid().x()+radius*cos(angleMinusHalfPi)+sigmaX, f->centroid().y()+radius*sin(angleMinusHalfPi)+sigmaY);
        if(!foundIntersection(*p, node))
            return p;
        delete p;
    //}
    return NULL;
}

bool AStarAlgorithm::foundIntersection(SLAM::Geometry::Point endPoint, const PathNode *node)
{

    LineSegment line(Point(node->getX(), node->getY()), endPoint);
    double rad = 0; //SAFETY_RADIUS/2;
    QRectF rect(endPoint.x()-rad, endPoint.y()+rad, 2*rad, 2*rad);
    foreach(const LineSegment *ls, slamMap->walls()){
        if(line.intersects(*ls))
            return true;
        if(rectContainsWall(ls, rect)){
            return true;
        }
    }
    return false;
}



bool AStarAlgorithm::rectContainsWall(const LineSegment *wall, const QRectF &rect)
{
    if (rect.contains(wall->x1(), wall->y1()) || rect.contains(wall->x2(), wall->y2()))
        return true;
    if(numberOfIntersectingSegmentsWithAGivenRectangleBuiltByExpandingAPoint(wall, rect) > 0)
        return true;
    return false;
}

int AStarAlgorithm::numberOfIntersectingSegmentsWithAGivenRectangleBuiltByExpandingAPoint(const LineSegment *ls, const QRectF &rect)
{
   qreal x, y, w, h;
   rect.getRect(&x, &y, &w, &h);
   ldbg << "x = "<< x<< "; y = "<< y << "; w = "<< w<< "; h = "<< h<<endl;
   QList<LineSegment> sides;
   sides.append(LineSegment(x,y,x+w,y));
   sides.append(LineSegment(x,y,x,y-h));
   sides.append(LineSegment(x+w,y,x+w,y-h));
   sides.append(LineSegment(x,y-h,x+w,y-h));
   int count = 0;
   foreach(LineSegment l, sides){
       if(l.intersects(*ls))
           count++;
   }
   return count;
}

void AStarAlgorithm::printPath(AStarNode *node)
{
    ldbg << "==== PATHNODE PRINT ====" << endl;
    ldbg << "Node: <" << node->getPose()->getX() << ", " << node->getPose()->getY() << ">" << endl;
    ldbg << "list of visible frontiers:" << endl;
    if(node->getPose()->visibleFrontiers().size() == 0)
        ldbg << "NO visible frontiers" << endl;
    else{
        foreach(Frontier *f, node->getPose()->visibleFrontiers())
            ldbg << "Frontier centroid: <" << f->centroid().x() << ", " << f->centroid().y() << ">" << endl;
    }
    ldbg << "==== END PATHNODE PRINT ====" << endl;
}

void AStarAlgorithm::printPlan(AStarNode *actual, AStarNode *next, Data::Action *rotAction, Data::Action *traAction)
{
    ldbg << "Actual: <" << actual->getPose()->getX() << ", " << actual->getPose()->getY() << ", "<< actual->getPose()->getTheta() <<">" << endl;
    if(rotAction != NULL)
        ldbg << "Required Rotation: " << fromDegreeToRadiants(rotAction->getValue()) << endl;
    if(traAction != NULL)
        ldbg << "Required Traslation: " << traAction->getValue() << endl;
    ldbg << "Next: <" << next->getPose()->getX() << ", " << next->getPose()->getY() << ", "<< next->getPose()->getTheta() <<">" << endl;
}

}
