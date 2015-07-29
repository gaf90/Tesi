#include "hybridastaralgorithm.h"
#include "hybridposeaction.h"
#include "data/action.h"
#include "shared/config.h"

//#define TESTING_PP

namespace PathPlanner {
const double HybridAStarAlgorithm::aStarWeight = 1;//2
bool operator==(const HybridAStarAlgorithm::CellIndex &ci1, const HybridAStarAlgorithm::CellIndex &ci2)
{
    return ci1.x == ci2.x && ci1.y == ci2.y && ci1.theta == ci2.theta;
}

uint qHash(const HybridAStarAlgorithm::CellIndex &ci)
{
    return ci.x * ci.x * 17 + ci.y * ci.y * 13 + ci.theta * ci.theta * 103;
}

HybridAStarAlgorithm::HybridAStarAlgorithm(bool aisKenaf)
{

//    availableActions.append(HybridAStarAction(MAX_SPEED*TOP_SPEED_PERCENTAGE,
//                                              MAX_SPEED*TOP_SPEED_PERCENTAGE)); //high speed
//    availableActions.append(HybridAStarAction(MAX_SPEED*TOP_SPEED_PERCENTAGE*ROTATION_SPEED_MAIN_WHEEL_PERCENTAGE,
//                                              MAX_SPEED*TOP_SPEED_PERCENTAGE*ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE)); //high front right
//    availableActions.append(HybridAStarAction(MAX_SPEED*TOP_SPEED_PERCENTAGE*ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE,
//                                              MAX_SPEED*TOP_SPEED_PERCENTAGE*ROTATION_SPEED_MAIN_WHEEL_PERCENTAGE)); //high front left

    isKenaf = aisKenaf;
    availableActions.append(HybridAStarAction(MAX_SPEED */* WHEEL_RADIUS */ TOP_SPEED_PERCENTAGE,
                                        MAX_SPEED * /* WHEEL_RADIUS */ TOP_SPEED_PERCENTAGE)); //front
    availableActions.append(HybridAStarAction(MAX_SPEED * /* WHEEL_RADIUS */ ROTATION_SPEED_MAIN_WHEEL_PERCENTAGE,
                                        MAX_SPEED /* WHEEL_RADIUS */* ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE)); //front right
    availableActions.append(HybridAStarAction(MAX_SPEED /* WHEEL_RADIUS */* ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE,
                                        MAX_SPEED /* WHEEL_RADIUS */* ROTATION_SPEED_MAIN_WHEEL_PERCENTAGE)); //front left

    availableActions.append(HybridAStarAction(MAX_SPEED /* WHEEL_RADIUS */* TOP_SPEED_PERCENTAGE,
                                        -MAX_SPEED /* WHEEL_RADIUS */* TOP_SPEED_PERCENTAGE)); //rotate at top speed
    availableActions.append(HybridAStarAction(-MAX_SPEED /* WHEEL_RADIUS */* TOP_SPEED_PERCENTAGE,
                                            MAX_SPEED /* WHEEL_RADIUS */* TOP_SPEED_PERCENTAGE)); //rotate at top speed

    availableActions.append(HybridAStarAction(MAX_SPEED /* WHEEL_RADIUS */ * ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE,
                                        -MAX_SPEED /* WHEEL_RADIUS */ * ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE)); //rotate at low speed
    availableActions.append(HybridAStarAction(-MAX_SPEED /* WHEEL_RADIUS */ * ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE,
                                            MAX_SPEED /* WHEEL_RADIUS */ * ROTATION_SPEED_SECONDARY_WHEEL_PERCENTAGE)); //rotate at low speed
}

QStack<AbstractAction *> *HybridAStarAlgorithm::computePath(const Data::Pose &startingPose,
                       const Data::Pose &destinationPose, SLAM::Map *aSlamMap, bool aNeedOrientation)
{
    ldbg << endl << endl << "starting new pathplanner calculus " << endl;
    needOrientation = aNeedOrientation;
    slamMap = aSlamMap;
    QQueue<PathPlanner::HybridAStarNode> nodes = calculateNodes(startingPose, destinationPose);
    printWalls(slamMap->walls(), "Walls");
    printStartDestPoints(startingPose, destinationPose);
    printPath(nodes);
    ldbg << "plot(xWalls, yWalls, 'r', xStartDest(1), yStartDest(1), 'og', xStartDest(2), yStartDest(2), 'xg', xPath, yPath, 'b');" << endl;
    printPoints(testedPoses, "TestedPoints");
    ldbg << "plot(xWalls, yWalls, 'r', xStartDest(1), yStartDest(1), 'og', xStartDest(2), yStartDest(2), 'xg', xPath, yPath, 'b', xTestedPoints, yTestedPoints,'ob');" << endl;
    return getPoseFromNodes(nodes);
}

QQueue<PathPlanner::HybridAStarNode> HybridAStarAlgorithm::calculateNodes(
        const Data::Pose &startingPose, const Data::Pose &destinationPose)
{
    ldbg << "starting pose is " << startingPose << ", destination pose is " << destinationPose << endl;
    QQueue<HybridAStarNode> pathNodes;
    QList<HybridAStarNode> nodes;
    QMultiMap<double, HybridAStarNode *> frontiers;
    HybridAStarNode * closestNode; //the closest node to the goal
    bool pathFound = false;
    HybridAStarNode *initialNode, *tempNode, *frontierNode, *finalNode;
    QMultiMap<double, HybridAStarNode *>::iterator it;
    QSet<CellIndex> visitedCells; //first int is the row, QList contains the list of the visited cols in the row
    QHash<HybridAStarNode *, HybridAStarNode *> previousNodeQHash; //first node is reached from the second node
    initialNode = new HybridAStarNode(startingPose, HybridAStarAction(0, 0), 0);
    closestNode = initialNode;
    frontiers.insert(0 + heuristicDistance(startingPose, destinationPose), initialNode);
    int i = 0, notSafeMovement = 0;
    ldbg << "frontiers.size() = " << frontiers.size() << endl;


    while(frontiers.size() > 0 && i < PP_MAX_ITERATIONS) {
//        if(i%100 == 0)
//            ldbg << "LOG = iterations passed: " << i << endl;
        i++;
        it = frontiers.begin();
        frontierNode = it.value();
        ldbg << endl << "analyzing node " << frontierNode->getPose() << " with value " << it.key() << endl;

        if(frontierNode != initialNode &&
                !canReach(*previousNodeQHash[frontierNode], *frontierNode)) {
//            ldbg << "node " << frontierNode->getPose() << " is not Safe" << endl;
            notSafeMovement++;
        } else {
            if(frontierNode->getPose().getDistance(destinationPose) <
                    DISTANCE_FOR_DESTINATION_POINT_REACHED) {
                finalNode = frontierNode;
                pathFound = true;
                break;
            }

            for(int i = 0; i < availableActions.size(); i++){
                HybridAStarAction tempAction = availableActions[i];
                ldbg << "action " << i << endl;
                tempNode = calculateNextNode(*frontierNode, tempAction);

                if(isAlreadyVisitedCell(*tempNode, visitedCells))
                    continue;
                nodes.append(tempNode);
                Data::Pose tempPose = tempNode->getPose();
                frontiers.insert(tempNode->getDistance() +
                                 aStarWeight * heuristicDistance(tempPose, destinationPose), tempNode);
                previousNodeQHash.insert(tempNode, frontierNode);
                testedPoses.append(tempNode->getPose());
                if(tempPose.getDistance(destinationPose) < closestNode->getPose().getDistance(destinationPose))
                    closestNode = tempNode;
                ldbg << "creating node at " << tempPose << " with value "
                     << tempNode->getDistance() + heuristicDistance(tempPose, destinationPose) << endl;
            }
        }
        frontiers.remove(it.key(), it.value());
    }

    if(pathFound) {
        ldbg << "to find a path between two points distant " << startingPose.getDistance(destinationPose) << " meters "
             << i << " iterations were necessary" << endl;
        tempNode = finalNode;
    }else{
        ldbg << "path NOOOT found" << endl;
        ldbg << "max iterations " << (i == PP_MAX_ITERATIONS ? "not" : "") << " reached" << endl;
        ldbg << "walls used "<<slamMap->walls().size()<<endl;
        ldbg << "frontiers should be zero... "<<frontiers.size()<<endl;
        ldbg << "nodes tested are " << nodes.count() << endl;
        ldbg << "not safe movements found are " << notSafeMovement << endl;
//        tempNode = closestNode;
        return pathNodes;
    }
    pathNodes.enqueue(HybridAStarNode(tempNode));

    while(tempNode != initialNode){
        tempNode = previousNodeQHash[tempNode];
        pathNodes.enqueue(HybridAStarNode(tempNode));
    }

    /* Free Giusto Merda Garbage */
    for(QHash<HybridAStarNode *, HybridAStarNode *>::iterator it = previousNodeQHash.begin();
        it != previousNodeQHash.end(); ++it) {
        delete it.key();
    }

    return pathNodes;
}

double HybridAStarAlgorithm::heuristicDistance(const Data::Pose &currentPose, const Data::Pose &destinationPose)
{
    double rotationWeight = 0;
    double distanceWeight = currentPose.getDistance(destinationPose);
    rotationWeight = fabs(wrapRad(currentPose.getTheta() - destinationPose.getTheta())) * ROTATION_WEIGHT;

    return rotationWeight + distanceWeight/* + perpendicularWeight*/;
}

PathPlanner::HybridAStarNode * HybridAStarAlgorithm::calculateNextNode(PathPlanner::HybridAStarNode &oldNode, HybridAStarAction &action)
{
    const double delta = DELTA_T, widthRobot = WHEEL_BASE;
    const double leftVelocity = action.getVl(), rightVelocity = action.getVr();
    const Data::Pose oldPose = oldNode.getPose();
    double x, y, theta;

//    const double u1 = WHEEL_RADIUS/2*(leftVelocity+rightVelocity);
//    const double u2 = WHEEL_RADIUS/2/WHEEL_BASE*(rightVelocity-leftVelocity);

//    theta = oldPose.theta()+u2*delta;

//    double avTheta = (theta+oldPose.theta())/2;

//    x = oldPose.x() + u1*cos(avTheta)*delta;
//    y = oldPose.y() + u2*sin(avTheta)*delta;


    if(almostEqual(leftVelocity, rightVelocity)) {
        x = oldPose.x() - rightVelocity * delta * sin(oldPose.theta());
        y = oldPose.y() + rightVelocity * delta * cos(oldPose.theta());
        theta = oldPose.theta();
        ldbg << "straight movement, initial point " << oldPose.x() << "," << oldPose.y() << " - calculated point " << x << "," << y << " with theta = " << oldPose.theta()<< endl;
    } else {
        const double R = 0.5 * widthRobot * (rightVelocity + leftVelocity) / (rightVelocity - leftVelocity);
        const double w = (rightVelocity - leftVelocity) / widthRobot;
        x = oldPose.x() + R * cos(oldPose.theta() + w * delta) - R * cos(oldPose.theta());
        y = oldPose.y() + R * sin(oldPose.theta() + w * delta) - R * sin(oldPose.theta());
        theta = wrapRad(oldPose.theta() + w*delta);
        ldbg << "rotation movement, initial point " << oldPose.x() << "," << oldPose.y() << "," << oldPose.theta() << " - calculated point " << x << "," << y << "," << theta << endl;

    }

    Data::Pose newPose = Data::Pose(x, y, theta);
    return new HybridAStarNode(newPose, action, oldNode.getDistance() +
                               heuristicDistance(oldPose, newPose));
}

bool HybridAStarAlgorithm::isAlreadyVisitedCell(PathPlanner::HybridAStarNode &node, QSet<CellIndex> &visitedCells)
{
    CellIndex cell = calculateCell(node.getPose());
    if(visitedCells.contains(cell)){
        return true;
    } else {
        visitedCells.insert(cell);
        return false;
    }
}

bool HybridAStarAlgorithm::canReach(
        const PathPlanner::HybridAStarNode &prevNode, const PathPlanner::HybridAStarNode &currentNode)
{
    SLAM::boolplus canReach;
    if (isKenaf)
        canReach = slamMap->isReachable(SLAM::Geometry::Point(prevNode.getPose().getX(), prevNode.getPose().getY()),
                                                        SLAM::Geometry::Point(currentNode.getPose().getX(), currentNode.getPose().getY()),
                                                        KENAF_RADIUS);
    else
        canReach = slamMap->isReachable(SLAM::Geometry::Point(prevNode.getPose().getX(), prevNode.getPose().getY()),
                                                    SLAM::Geometry::Point(currentNode.getPose().getX(), currentNode.getPose().getY()),
                                                    P3AT_RADIUS);
//    ldbg << "boolplus = " << canReach << endl;
    if (canReach == SLAM::False){
        return false;
    } if (canReach == SLAM::Maybe){
        return PASS_THROUGH_UNKNOWN_AREAS_ALLOWED;
    } else {
        return true;
    }
}

HybridAStarAlgorithm::CellIndex HybridAStarAlgorithm::calculateCell(Data::Pose pose)
{
    CellIndex cell;
    cell.x = floor(pose.x() / CELL_SIDE);
    cell.y = floor(pose.y() / CELL_SIDE);
    cell.theta = (((int) floor((pose.theta() - M_PI / 8) / M_PI_4) + 1) + 4) % 8;
    return cell;
}

QStack<AbstractAction *> *HybridAStarAlgorithm::getPoseFromNodes(QQueue<PathPlanner::HybridAStarNode> &nodes)
{

    QStack<AbstractAction *> *poses = NULL;
    if(nodes.isEmpty())
        return poses;

    poses = new QStack<AbstractAction *>;
    while(!nodes.isEmpty()) {
        poses->push(new HybridPoseAction(nodes.dequeue().getPose()));
    }
    return poses;
}


void HybridAStarAlgorithm::printWalls(const QList<SLAM::Geometry::LineSegment *> & walls, QString varName)
{
    QString xIni ="" , yIni="", xFin="", yFin="";
    for(int i=0; i < walls.size(); i++){
        xIni.append(" ").append(QString::number(walls.at(i)->x1(), 'f', 4));
        yIni.append(" ").append(QString::number(walls.at(i)->y1(), 'f', 4));
        xFin.append(" ").append(QString::number(walls.at(i)->x2(), 'f', 4));
        yFin.append(" ").append(QString::number(walls.at(i)->y2(), 'f', 4));
    }
    ldbg << "x"<<varName.toLatin1()<<"=[" << xIni << ";" << xFin << "];" << endl;
    ldbg << "y"<<varName.toLatin1()<<"=[" << yIni << ";" << yFin << "];" << endl;
}

void HybridAStarAlgorithm::printStartDestPoints(const Data::Pose &startingPose, const Data::Pose &destinationPose)
{
    ldbg << "xStartDest=[" << QString::number(startingPose.getX(), 'f', 4) << " " << QString::number(destinationPose.getX(), 'f', 4) << "];" << endl;
    ldbg << "yStartDest=[" << QString::number(startingPose.getY(), 'f', 4) << " " << QString::number(destinationPose.getY(), 'f', 4) << "];" << endl;
}

void HybridAStarAlgorithm::printPoints(const QList<Data::Pose> &poses, QString varName)
{
    QString x = "";
    QString y = "";
    for(int i = 0; i < poses.size(); i++){
        x.append(" ").append(QString::number(poses.at(i).x(), 'f', 4));
        y.append(" ").append(QString::number(poses.at(i).y(), 'f', 4));
    }
    ldbg << "x" << varName << "=[" << x << "];" << endl;
    ldbg << "y" << varName << "=[" << y << "];" << endl;
}

void HybridAStarAlgorithm::printPath(QList<PathPlanner::HybridAStarNode> path)
{
    if(path.size() == 0){
        ldbg << "xxx Path not found" << endl;
        return;
    }
    QString xIni ="" , yIni="", xFin="", yFin="", vr="", vl="", thetas="";
    PathPlanner::HybridAStarNode node;
    Data::Pose oldPose;
    Data::Pose newPose;
    node = path.at(0);
    oldPose = node.getPose();
    for(int i=1; i < path.size(); i++){
        node = path.at(i);
        newPose = node.getPose();
        xIni.append(" ").append(QString::number(oldPose.getX(), 'f', 4));
        yIni.append(" ").append(QString::number(oldPose.getY(), 'f', 4));
        xFin.append(" ").append(QString::number(newPose.getX(), 'f', 4));
        yFin.append(" ").append(QString::number(newPose.getY(), 'f', 4));
        vl.append(" ").append(QString::number(node.getAction().getVl(), 'f', 4));
        vr.append(" ").append(QString::number(node.getAction().getVr(), 'f', 4));
        thetas.append(" ").append(QString::number(node.getPose().theta(), 'f', 4));
        oldPose = newPose;
    }
    ldbg << "xPath=[" << xIni << ";" << xFin << "];" << endl;
    ldbg << "yPath=[" << yIni << ";" << yFin << "];" << endl;
    ldbg << "vr = [" << vr << "];" << endl;
    ldbg << "vl = [" << vl << "];" << endl;
    ldbg << "th = [" << thetas << "];" << endl;
}


QStack<PathPlanner::AbstractAction *> * HybridAStarAlgorithm::getOldStylePoseFromNodes(QQueue<PathPlanner::HybridAStarNode> &nodes)
{
    //in nodes the first node is the destination node, the last is the closet to the starting pose
    QStack<PathPlanner::AbstractAction *> * actions = NULL;
    if(nodes.size() == 0){
        ldbg << "xxx Path not found" << endl;
        return actions;
    }
    QList<Data::Pose *> poses;
    bool safeMovementNotFound = false;
    int currentNodeIndex = nodes.size()-1;
    int tempNodeIndex;
    int movementsRequired = 0;
    poses.append(new Data::Pose(nodes.at(nodes.size()-1).getPose()));
    while((currentNodeIndex > 0 /*there are other poses to add*/) && (safeMovementNotFound == false)){
        movementsRequired++;
        for(tempNodeIndex = 0; tempNodeIndex <= currentNodeIndex; tempNodeIndex++){
            if(canReach(nodes.at(currentNodeIndex),nodes.at(tempNodeIndex))){
               poses.append(new Data::Pose(nodes.at(tempNodeIndex).getPose()));
               currentNodeIndex = tempNodeIndex;
               break;
            }
            if(tempNodeIndex == currentNodeIndex){
                safeMovementNotFound = true;
                break;
            }
        }
    }
    for(int i=0; i < poses.size(); i++){
        ldbg << "pose " << i << "is " << poses.at(i) << endl;
    }
    if(safeMovementNotFound == true){
        ldbg << "dude, there is an error somewhere, it was not possible to find a safe movement from a node to a following one without crashing into a wall, this is NOT supposed to happen" << endl;
        ldbg << "however, there were " << poses.size() << " poses of the final path found" << endl;
        for(int i=0; i<poses.size();i++){
            ldbg << "pose " << i << ": " << poses.at(i) << endl;
        }
        return actions;
    }

    ldbg << "moving in the old style way we need to do " << movementsRequired << "submovements" << endl;
    actions = new QStack<PathPlanner::AbstractAction *>;
    QList<Data::Pose *> reorientedPoses;
    reorientedPoses.append(poses.first());
    double translation, rotationRadiant, rotationDegree;
    for(int i = 0; i < poses.size()-1; i++){
        translation = reorientedPoses.at(i)->getDistance(*poses.at(i+1));
        rotationRadiant = wrapRad(computeRotationFromPoses(*reorientedPoses.at(i), *poses.at(i+1)));
        rotationDegree = fromRadiantToDegree(rotationRadiant);
        actions->push(new Data::Action(Data::Action::Rotation, rotationDegree));
        actions->push(new Data::Action(Data::Action::Translation, translation));
        ldbg << "to go from " << *reorientedPoses.at(i) << " to " << *poses.at(i+1) << " you have to perform a rotation of "
             << rotationDegree << " (in radiants: " << rotationRadiant << ") and a translation of " << translation << endl;
        reorientedPoses.append(new Data::Pose(poses.at(i+1)->x(), poses.at(i+1)->y(), wrapRad(reorientedPoses.at(i)->theta()+rotationRadiant)));
    }

    QStack<PathPlanner::AbstractAction *> * invertedActions = new QStack<PathPlanner::AbstractAction *>;
    while(actions->size() > 0)
        invertedActions->push(actions->pop());

    //---only for debug
    QList<SLAM::Geometry::LineSegment *> movements;
    for(int i = 0; i < poses.size()-1; i++){
        movements.append(new SLAM::Geometry::LineSegment(poses.at(i)->getX(),poses.at(i)->getY(),
                                                         poses.at(i+1)->getX(),poses.at(i+1)->getY()));
    }
    printWalls(movements, "OldStylePath");
    foreach(SLAM::Geometry::LineSegment *l, movements)
        delete l;

    foreach(Data::Pose *p, poses)
        delete p;

    foreach(Data::Pose *p, reorientedPoses)
        delete p;

    return invertedActions;
}

void HybridAStarAlgorithm::loadWalls()
{
    using namespace Data;
    QList<SLAM::Geometry::Point> points;
    points.append(SLAM::Geometry::Point(2,0));
    points.append(SLAM::Geometry::Point(4,0));
    points.append(SLAM::Geometry::Point(4,2));
    points.append(SLAM::Geometry::Point(6,2));
    points.append(SLAM::Geometry::Point(6,1));
    points.append(SLAM::Geometry::Point(5,1));
    points.append(SLAM::Geometry::Point(5,0));
    points.append(SLAM::Geometry::Point(8,0));
    points.append(SLAM::Geometry::Point(8,4));
    points.append(SLAM::Geometry::Point(9,4));
    points.append(SLAM::Geometry::Point(9,2));
    points.append(SLAM::Geometry::Point(10,2));
    points.append(SLAM::Geometry::Point(10,5));
    points.append(SLAM::Geometry::Point(14,5));
    points.append(SLAM::Geometry::Point(14,10));
    points.append(SLAM::Geometry::Point(15,10));
    points.append(SLAM::Geometry::Point(15,5));
    points.append(SLAM::Geometry::Point(16,5));
    points.append(SLAM::Geometry::Point(16,7));
    points.append(SLAM::Geometry::Point(19,7));
    points.append(SLAM::Geometry::Point(19,8));
    points.append(SLAM::Geometry::Point(16,8));
    points.append(SLAM::Geometry::Point(16,11));
    points.append(SLAM::Geometry::Point(17,11));
    points.append(SLAM::Geometry::Point(17,12));
    points.append(SLAM::Geometry::Point(15,12));
    points.append(SLAM::Geometry::Point(15,16));
    points.append(SLAM::Geometry::Point(7,16));
    points.append(SLAM::Geometry::Point(7,14));
    points.append(SLAM::Geometry::Point(5,14));
    points.append(SLAM::Geometry::Point(5,11));
    points.append(SLAM::Geometry::Point(0,11));
    points.append(SLAM::Geometry::Point(0,3));
    points.append(SLAM::Geometry::Point(2,3));

    createWalls(points);


    points.clear();
    points.append(SLAM::Geometry::Point(1,4));
    points.append(SLAM::Geometry::Point(7,4));
    points.append(SLAM::Geometry::Point(7,5));
    points.append(SLAM::Geometry::Point(3,5));
    points.append(SLAM::Geometry::Point(3,7));
    points.append(SLAM::Geometry::Point(7,7));
    points.append(SLAM::Geometry::Point(7,12));
    points.append(SLAM::Geometry::Point(8,12));
    points.append(SLAM::Geometry::Point(8,13));
    points.append(SLAM::Geometry::Point(6,12));
    points.append(SLAM::Geometry::Point(6,9));
    points.append(SLAM::Geometry::Point(1,9));

    createWalls(points);


    points.clear();
    points.append(SLAM::Geometry::Point(10,7));
    points.append(SLAM::Geometry::Point(13,7));
    points.append(SLAM::Geometry::Point(13,9));
    points.append(SLAM::Geometry::Point(12,9));
    points.append(SLAM::Geometry::Point(12,10));
    points.append(SLAM::Geometry::Point(13,10));
    points.append(SLAM::Geometry::Point(13,12));
    points.append(SLAM::Geometry::Point(14,12));
    points.append(SLAM::Geometry::Point(14,15));
    points.append(SLAM::Geometry::Point(8,15));
    points.append(SLAM::Geometry::Point(8,14));
    points.append(SLAM::Geometry::Point(9,14));
    points.append(SLAM::Geometry::Point(9,12));
    points.append(SLAM::Geometry::Point(10,12));
    points.append(SLAM::Geometry::Point(10,11));
    points.append(SLAM::Geometry::Point(8,11));
    points.append(SLAM::Geometry::Point(8,9));
    points.append(SLAM::Geometry::Point(10,9));

    createWalls(points);
}

void HybridAStarAlgorithm::createWalls(QList<SLAM::Geometry::Point> points)
{
    int i;
    for(i=1; i < points.size(); i++){
        walls.append(new SLAM::Geometry::LineSegment(points.at(i-1),points.at(i)));
    }
    walls.append(new SLAM::Geometry::LineSegment(points.at(i-1),points.at(0)));
}

bool HybridAStarAlgorithm::canFakeReach(Data::Pose startingPose, Data::Pose destinationPose)
{
    SLAM::Geometry::LineSegment movement(startingPose.getX(), startingPose.getY(),
                                         destinationPose.getX(), destinationPose.getY());
    foreach(SLAM::Geometry::LineSegment * wall, walls){
        if(movement.intersects(*wall))
            return false;
    }
    return true;
}
}



