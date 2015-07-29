#include "pathplannermodule.h"
#include "slam/geometry/frontier.h"
#include "pathPlanner/aStar/astaralgorithm.h"
#include "pathplannerutils.h"
#include "shared/constants.h"
#include "pathPlanner/hybridAStar/hybridastaralgorithm.h"
#include "pathPlanner/RRT/rrtalgorithm.h"
#include "pathPlanner/abstractaction.h"
#include "hybridAStar/hybridposeaction.h"
#include "semanticMapping/doorcreator.h"
#include "semanticMapping/geometry/doorlinesegment.h"
#include "data/buddymessage.h"
#include "data/errornotificationmessage.h"
#include <typeinfo>


namespace PathPlanner{

using namespace SLAM;
using namespace Geometry;
using namespace Shared;
using namespace Data;

PathPlannerModule::PathPlannerModule(uint identifier, bool isKenaf, QObject *parent) :
    QThread(parent),
    identifier(identifier),
    poseQueue(new MutexQueue<Pose *>()),
    pathPlannerMutex(new QMutex()),
    endComputation(false),
    enabled(true),
    pathPlannerHelper(this),
    isKenaf(isKenaf)

{
    connect(this, SIGNAL(sigPathNotFound()), &pathPlannerHelper, SLOT(sendMessageToGUI()), Qt::QueuedConnection);
}


PathPlannerModule::~PathPlannerModule()
{
    stopComputation();
    delete pathPlannerMutex;

    while(poseQueue->size()!=0){
        Pose * pose = poseQueue->dequeue();
        delete pose;
    }
    delete poseQueue;
}

void PathPlannerModule::setSlamModule(SLAM::SLAMModule *slamModule)
{
    this->slamModule = slamModule;
}

void PathPlannerModule::setRobotState(Data::RobotState *robotState)
{
    this->robotState = robotState;
}

void PathPlannerModule::changeStatus(bool enable)
{
    enablingMutex.lock();
    this->enabled = enable;
    enablingMutex.unlock();
}

void PathPlannerModule::computePath(const Pose &pose, const SLAM::Map &map)
{
    enablingMutex.lock();
    bool shouldContinue = enabled;
    ldbg<<"shouldContinue"<<shouldContinue<<endl;
    bool pathFound = false;
    enablingMutex.unlock();
    if(!shouldContinue)
        return;

    //Check if the point is reachable
    Point posePoint(pose.getX(), pose.getY());
    if(lastPoint.distance(posePoint) < ROBOT_DIAG/2 &&
            fabs(robotState->getLeftSpeed())>0.1)
    {
        ldbg << "### path to the same last point" << endl;
        return; //I'm already in that point or I'm going toward it;
    }
    lastPoint = Point(posePoint);

    bool existFrontierNearPoint = false/*, isFrontier = false, isVictim = false*/;

    const QList<Frontier *> frontiers = map.frontiers();

    existFrontierNearPoint = frontierFound(frontiers, posePoint);

    if(!pathFound)
    {
        //ldbg << "let's try with AThrun" << endl;
        emit sigStopRobotForPlanningPM();
        HybridAStarAlgorithm *hybridAlgorithm = new HybridAStarAlgorithm(isKenaf);
        const Data::Pose *myPose = map.lastRobotPose(identifier);
        ldbg << identifier << " myPose = ("<<myPose->getX()<<", "<<myPose->getY()<<", "<<myPose->getTheta()<<")"<<endl;
        slamMap = map;
        QStack<AbstractAction *> *hybridActionStack = hybridAlgorithm->computePath(*myPose, pose, &slamMap);
        //Check if it should send actions to the pathplanner
        enablingMutex.lock();
        bool shouldContinue = enabled;
        enablingMutex.unlock();
        //pathplanning stopped
        if(!shouldContinue && hybridActionStack != NULL){
            while(!hybridActionStack->isEmpty())
                delete hybridActionStack->pop();
            delete hybridActionStack;
            delete hybridAlgorithm;
            return;
        }
        if(hybridActionStack != NULL){
            ldbg << "pass the actions computed to the robot" << endl;

            QList<Pose> obt;
            obt = postProcessHybridAStar(hybridActionStack, map);

            if(!obt.isEmpty()){


                Point myPoint(myPose->getX(), myPose->getY());
                Point firstObtPoint(obt[1].getX(), obt[1].getY());
                Point lastObtPoint(obt[0].getX(), obt[0].getY());

                double distMyEnd = myPoint.distance(lastObtPoint);
                double distObtPoints = firstObtPoint.distance(lastObtPoint);

                ldbg << "=====> Recompute interpolated Path <=====" << endl;

                QStack<AbstractAction *> * firstActions = NULL;
                Pose p;
                if(distObtPoints < distMyEnd){
                    firstActions = hybridAlgorithm->computePath(*myPose, obt.at(1), &slamMap);
                    if(firstActions == NULL){
                        while(!hybridActionStack->isEmpty()){
                            AbstractAction *action = hybridActionStack->pop();
                            delete action;
                        }
                        delete hybridActionStack;

                        //Notify the failure
                        ldbg << "PathPlannerThreadID = " << QThread::currentThreadId() << endl;
                       emit sigPathNotFound();


                        //Signal a bad frontier
                        ldbg << "BAD_FRONT : emitting bad frontier signal" << endl;
                        emit sigHandleBandleFrontierPM(pose);
                        return;
                    }

                    HybridPoseAction * action = (HybridPoseAction *) firstActions->first();
                    double newTheta = atan2(obt[0].y()-action->getValue().y(), obt[0].x()-action->getValue().x())-M_PI_2;
                    p = Pose(action->getValue().x(), action->getValue().y(), newTheta);
                } else {
                    double newTheta = atan2(obt[0].y()-myPose->y(), obt[0].x()-myPose->x())-M_PI_2;
                    p = Pose(myPose->x(), myPose->y(), newTheta);
                }


                QStack<AbstractAction *> * midActions = hybridAlgorithm->computePath(p, obt.at(0), &slamMap);
                if(midActions == NULL){
                    //Clean the old stack
                    while(!hybridActionStack->isEmpty()){
                        AbstractAction *action = hybridActionStack->pop();
                        delete action;
                    }
                    delete hybridActionStack;

                    emit sigPathNotFound();

                    //Signal a bad frontier
                    ldbg << "BAD_FRONT : emitting bad frontier signal" << endl;
                    emit sigHandleBandleFrontierPM(pose);
                    return;
                }


                QStack<AbstractAction *> * lastActions = hybridAlgorithm->computePath(((HybridPoseAction *)midActions->first())->getValue(),                                                                                      pose, &slamMap);
                QStack<AbstractAction *> tmpStack;

                if(firstActions != NULL){
                    //On top I have the first action I should do
                    while(!firstActions->isEmpty()){
                        tmpStack.push(firstActions->pop());
                    }
                    //In tmpStack, on top, i have the last action I should do.
                    delete firstActions;
                }
                //I put a rotation as last action to do


                if(midActions != NULL){
                    //on top of midactions I have the first action I should do
                    while(!midActions->isEmpty()){
                        tmpStack.push(midActions->pop());
                    }
                    //In tmp stack I have, on top, the last action I should do
                    delete midActions;
                }

                if(lastActions != NULL){
                    //I put in lastAction all the actions I should do
                    //reverting the order.
                    while(!tmpStack.isEmpty()){
                        lastActions->push(tmpStack.pop());
                    }
                    //Now, on top, I should have the first action of the whole plan.

                    //Clean the old path
                    while(!hybridActionStack->isEmpty()){
                        AbstractAction *action = hybridActionStack->pop();
                        delete action;
                    }
                    delete hybridActionStack;
                    hybridActionStack = lastActions;
                }

                //ldbg << "=====><=====" << endl;
            }


            QString xIni ="" , yIni="";
            for(int i=hybridActionStack->size()-1; i>= 0; i--){
                HybridPoseAction *act = (HybridPoseAction *) hybridActionStack->at(i);
                xIni.append(" ").append(QString::number(act->getValue().x(), 'f', 4));
                yIni.append(" ").append(QString::number(act->getValue().y(), 'f', 4));
            }
            ldbg << "xCont=[" << xIni << "];" << endl;
            ldbg << "yCont=[" << yIni << "];" << endl;

            QString xSent ="" , ySent="";

            enablingMutex.lock();
            bool shouldContinue = enabled;
            enablingMutex.unlock();
            //pathplanning stopped
            if(!shouldContinue){
                while(!hybridActionStack->isEmpty())
                    delete hybridActionStack->pop();
                delete hybridActionStack;
                delete hybridAlgorithm;
                return;
            }

            emit sigFrontierToReachPM(pose);

            //The percentage of the plan I want to send
            int percentageToSend = hybridActionStack->size()*0.5;
            while(hybridActionStack->size() > percentageToSend){
                AbstractAction *action = hybridActionStack->pop();
                HybridPoseAction *act = (HybridPoseAction *)action;
                xSent.append(" ").append(QString::number(act->getValue().x(), 'f', 4));
                ySent.append(" ").append(QString::number(act->getValue().y(), 'f', 4));
                emit sigPerformActionPM(action);
                delete action;
            }

            emit sigCleanBadFrontiersPM();
            //}

            ldbg << "xSent=[" << xSent << "];" << endl;
            ldbg << "ySent=[" << ySent << "];" << endl;
            ldbg << "hold on;"<<endl;
            ldbg << "plot(xSent, ySent, 'c');"<<endl;


            while(!hybridActionStack->isEmpty()){
                AbstractAction *action = hybridActionStack->pop();
                delete action;
            }
            emit sigRestartExplorationPM();
            ldbg << "Delete the action stack" << endl;
            delete hybridActionStack;
        } else {
            ldbg << "Also Thrun can fail!!!" << endl;
            ldbg << "We lose! >__< " << endl;

            emit sigPathNotFound();
            //Signal a bad frontier
            ldbg << "BAD_FRONT : emitting bad frontier signal" << endl;
            emit sigHandleBandleFrontierPM(pose);
        }
        delete hybridAlgorithm;
    } else {


    }

}

void PathPlannerModule::onPointToReachPP(double x, double y)
{
    Pose *pose = new Pose(x, y, 0.0);
    poseQueue->enqueue(pose);
}

void PathPlannerModule::run(){
    while(true){
        bool shouldReturn = false;
        pathPlannerMutex->lock();
        shouldReturn = endComputation;
        //ldbg << "Path planner shold stop its computation" << endl;
        pathPlannerMutex->unlock();

        if(shouldReturn)
            return;

        ldbg << "wait for a new pose to reach" << endl;
        const Pose *poseToReach = poseQueue->dequeue();
        ldbg << "New Pose to reach" << endl;
        computePath(*poseToReach, slamModule->getMap(true));
        ldbg << "Path to the pose computed!" << endl;
        delete poseToReach;

    }
}

void PathPlannerModule::stopComputation()
{
    pathPlannerMutex->lock();
    endComputation = true;
    pathPlannerMutex->unlock();
}

void PathPlannerModule::postProcessPlan(QStack<AbstractAction*>* plan)
{
    ldbg << "Postprocessing started. Starting plan size: "<<plan->size()<<endl;
    //ldbg << "first action to perform: "<<plan->top()->getType()<< endl;
    QStack<AbstractAction *> *tempStack = new QStack<AbstractAction*>();
    bool translationFound = false;
    bool hybridAction = false;
    while(!plan->isEmpty()){
        Data::Action *act1 = NULL;
        if(typeid(plan->top()) == typeid(HybridPoseAction*)){
            hybridAction = true;
            break;
        }

        act1 = (Data::Action *)plan->pop();

        if(!plan->isEmpty()){
            Data::Action *act2 = (Data::Action *)plan->top();
            while(!plan->isEmpty() && act2->getType() == act1->getType()){
                Data::Action *act2 = (Data::Action *)plan->pop();
                ldbg<<"newvalue"<<act1->getValue()+act2->getValue()<<endl;
                act1->setValue(act1->getValue()+act2->getValue());
                delete act2;
            }
        }
        if(act1->getType() == Data::Action::Translation)
            translationFound = true;
        tempStack->push(act1);
    }

    if(!hybridAction){
        while(!tempStack->isEmpty()){
            if(translationFound)
                plan->push(tempStack->pop());
            else
                delete tempStack->pop();
        }
    }

    delete tempStack;
    ldbg << "post processing is finished. Final plan size: "<<plan->size()<<endl;
    //ldbg << "first action to perform: "<<plan->top()->getType()<< endl;

}

double PathPlannerModule::estimateTimeForAPlan(PathPlannerTasks task, const Pose &pose)
{
    double toRet = 0.0;
    bool useAStar = false;
    Point posePoint(pose.getX(), pose.getY());
    if(task == FRONTIER){
        useAStar = frontierFound(slamModule->getMap(true).frontiers(), posePoint);
    }

    if(useAStar){
        const SLAM::Map map = slamModule->getMap(true);
        AStarAlgorithm *algorithm = new AStarAlgorithm(identifier, &map, pose.getX(), pose.getY());
        QStack<AbstractAction *> *actionStack = algorithm->doAlgorithm();
        //postProcessPlan(actionStack);
        if(actionStack!=NULL){
            while(!actionStack->isEmpty()){
                AbstractAction *act = actionStack->pop();
                toRet += act->getTimeEstimate();
                delete act;
            }
            delete actionStack;
        }
    } else {
        HybridAStarAlgorithm *hybridAlgorithm = new HybridAStarAlgorithm(isKenaf);
        const Data::Pose *myPose = slamModule->getMap(true).lastRobotPose(identifier);
        SLAM::Map map = slamModule->getMap(true);
        QStack<AbstractAction *> *hybridActionStack = hybridAlgorithm->computePath(*myPose, pose, &map);
        if(hybridActionStack != NULL){
            AbstractAction *act = hybridActionStack->top();
            toRet = act->getTimeEstimate() * hybridActionStack->size(); //all the actions require the same amount of time
            //Clean the stack
            while(!hybridActionStack->isEmpty()){
                delete hybridActionStack->pop();
            }
            delete hybridActionStack;
        }
        else
            toRet = -1;

    }
    return toRet;
}

QList<Pose> PathPlannerModule::postProcessHybridAStar(QStack<AbstractAction *> *plan, const SLAM::Map &map)
{
    //SemanticMapping::SemanticHandler handler(&map);
    SemanticMapping::DoorCreator handler(&map);
    QList<SemanticMapping::Geometry::DoorLineSegment> doors = handler.getDoors();
    int j = 0;
    int doorAt = -1;
    bool doorFound = false;
    for(int i=0; i<plan->size()-2; i++){
        j=i+1;
        if(plan->size()-1 >= j){
            HybridPoseAction *act1 = (HybridPoseAction *) plan->at(i);
            HybridPoseAction *act2 = (HybridPoseAction *) plan->at(j);
            LineSegment s(act1->getValue().x(), act1->getValue().y(), act2->getValue().x(), act2->getValue().y());
            for(int k=0; k<doors.size(); k++){
                if(doors.at(k).intersects(s, 1e-3)){
                    doorFound = true;
                    doorAt = j;
                    //Get pSegment
                    SemanticMapping::Geometry::DoorLineSegment dls = doors.at(k);
                    LineSegment pSeg = dls.getPSegment(2);
                    //compute the distance between act1 and psegment.p1
                    Point pSegP1 = pSeg.p1();
                    Point pSegP2 = pSeg.p2();
                    Point act1P(act1->getValue().x(), act1->getValue().y());
                    double distance1 = pSegP1.distance2(act1P);
                    //compute the distance between act1 and psegment.p2
                    double distance2 = pSegP2.distance2(act1P);
                    //keep the point at minimum distance pm
                    Point pm1, pm2;
                    if(distance1 < distance2){
                        pm1 = pSegP1;
                        pm2 = pSegP2;
                    } else {
                        pm1 = pSeg.p2();
                        pm2 = pSeg.p1();
                    }
                    //substitute act1.value with pm
                    AbstractAction *old = plan->at(i);
                    HybridPoseAction *action = new HybridPoseAction(Pose(pm1.x(), pm1.y(), act1->getValue().theta()));
                    plan->replace(i, action);
                    delete old;
                    //substitute act2.value with the other point of pSegm
                    old = plan->at(j);
                    action = new HybridPoseAction(Pose(pm2.x(), pm2.y(), act2->getValue().theta()));
                    plan->replace(j, action);
                    delete old;
                    act2 = action;

                    QList<Data::Pose> poses;
                    Pose pose1 = Pose(pm1.x(), pm1.y(), act1->getValue().theta());
                    Pose pose2 = Pose(pm2.x(), pm2.y(), act2->getValue().theta());
                    double rot = computeRotationFromPoses(pose1, pose2);
                    pose1.setTheta(pose1.theta()+rot);
                    poses.append(pose1);
                    poses.append(pose2);

                    return poses;
                }
            }
            if(doorFound)
                break;
        }
    }

    QList<Pose> emptypose;
    return emptypose;
}

void PathPlannerModule::sendMessageToGui()
{
    ldbg << "PPThread is: " << QThread::currentThreadId() << endl;
    ErrorNotificationMessage erMsg(ErrorNotificationMessage::PathPlanner);
    BuddyMessage bud(robotNameFromIndex(identifier), robotNameFromIndex(BASE_STATION_ID),
                     BuddyMessage::ErrorNotification, &erMsg);
    WirelessMessage wlMsg(&bud, WirelessMessage::MessageExchange);
    emit sigPathNotFoundPM(wlMsg);
}
}

