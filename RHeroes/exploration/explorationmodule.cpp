#include "explorationmodule.h"
#include "explorationconstants.h"
#include "mcdmfunction.h"
#include "aojrffunction.h"
#include "dummyfunction.h"
#include "semanticbasicfunction.h"
#include "shared/logger.h"
#include "shared/config.h"
#include "data/wirelessmessage.h"
#include "shared/utilities.h"
#include "newfrontierevaluator.h"
#include <cmath>
#include <typeinfo>
#include <QMetaType>
#include <QDebug>
#include "PRM/evaluationPRM/prmfunction.h"
#include "PRM/evaluationPRM/prmfunctionmcdm.h"
#include "PRM/evaluationPRM/prmfunctiontovar.h"


#if defined(Q_OS_WIN32)
#   define isnan _isnan
#elif defined(Q_OS_MACX)
#   define isnan std::isnan
#endif

using namespace SLAM;

namespace Exploration {

using namespace Data;
using namespace SLAM;
using namespace Geometry;

ExplorationModule::ExplorationModule(
        uint robotId, EvalType type, SLAM::SLAMModule *slamModule, Data::RobotState *state,
        SemanticMapping::SemanticMappingModule *semMapModule, Coordination::CoordinationModule *coordModule,
        QObject *parent) :
    QThread(parent), eType(type),
    active(true),
    robotId(robotId), slamModule(slamModule), mapRenewTimer(new QTimer()),
    robotState(state), semMapModule(semMapModule), coordModule(coordModule), stateMutex(new QMutex()),
    forceFrontier(false), badFrontiersCentroids(new QList<Point *>())
{
    //Exploration <--> Timer
    connect(this, SIGNAL(sigStartTimerEM()), this, SLOT(onStartTimer()), Qt::QueuedConnection);
    connect(this, SIGNAL(sigStopTimerEM()), this, SLOT(onStopTimer()), Qt::QueuedConnection);
    connect(mapRenewTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    createEvaluationFunction(type);

    //Exploration --> Coordination Module (Start new auction)
    connect(this, SIGNAL(sigStartNewAuctionEM(EvaluationRecords*, bool)), this->coordModule, SLOT(startNewAuction(Exploration::EvaluationRecords*, bool)), Qt::DirectConnection);
}

ExplorationModule::ExplorationModule(uint robotId, EvalType type, QObject *parent) :
    QThread(parent),
    active(true),
    robotId(robotId), slamModule(NULL), mapRenewTimer(new QTimer()),
    robotState(NULL), semMapModule(NULL), coordModule(NULL), stateMutex(new QMutex()),
    forceFrontier(false), badFrontiersCentroids(new QList<Point *>())
{
    //Timer --> Exploration
    connect(mapRenewTimer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    createEvaluationFunction(type);

}

ExplorationModule::~ExplorationModule()
{
    while(!badFrontiersCentroids->isEmpty()){
        delete badFrontiersCentroids->last();
        badFrontiersCentroids->removeLast();
    }
    delete badFrontiersCentroids;

    quit();
    delete stateMutex;
    delete evalFunction;
    emit sigStopTimerEM();
    mapRenewTimer->deleteLater();

}

void ExplorationModule::createEvaluationFunction(EvalType type)
{
    //Create the evaluation function w.r.t the type passed.
    switch(type)
    {
    case Dummy:
        evalFunction = new DummyFunction(robotId);
        ldbg << "Exploration Module: Dummy policy" <<endl;
        break;
    case AOJRF:
        evalFunction = new AOJRFFunction(robotId);
        ldbg << "Exploration Module: AOJRF policy"<<endl;
        break;
    case BasicSemantic:
        evalFunction = new SemanticBasicFunction(robotId);
        ldbg << "Exploration Module: Semantic policy"<<endl;
        break;
    case PRM:
        break;
    case MCDM:
        break;
    case Tovar:
        break;
    default:
        evalFunction = new MCDMFunction(robotId);
        ldbg << "Exploration Module: MCDM policy"<<endl;
        break;
    }
}

void ExplorationModule::start()
{
    mapRenewTimer->start(MAP_RENEW_TIME);
    QThread::start();
}

void ExplorationModule::setSLAMModule(SLAM::SLAMModule *slamModule)
{
    ldbg << "Exploration Module: Set Slam Module"<<endl;

    this->slamModule = slamModule;
    map = slamModule->getMap();
    //PRM
    ldbg << "Exploration Module: Policy is:" << Config::policy <<endl;

    if(Config::policy==PRM)
    {
        PRM::PRMAlgorithm* prm = slamModule->getPRM();
        ldbg << "Exploration Module: Start Planner PRM, set evaluation and connect modules."<<endl;

        plannerPRM = new PRM::PathPlannerPRM();
        evalFunction= new PRM::PRMFunction(prm,plannerPRM,robotId);
        connect(plannerPRM,SIGNAL(sigPerformActionPP(PathPlanner::AbstractAction*)),this, SLOT(onPerformActionEM(PathPlanner::AbstractAction*)), Qt::DirectConnection);
        connect(plannerPRM,SIGNAL(sigRestartExplorationPP()),this, SLOT(onRestartExplorationEM()));
        connect(plannerPRM,SIGNAL(sigFrontierToReachPP(Data::Pose)),this, SLOT(onFrontierToReachEM(Data::Pose)));
    }
    if(Config::policy==Tovar)
    {
        PRM::PRMAlgorithm* prm= slamModule->getPRM();

        plannerPRM=new PRM::PathPlannerPRM();
        evalFunction= new PRM::PRMFunctionTovar(prm,plannerPRM,robotId);
        connect(plannerPRM,SIGNAL(sigPerformActionPP(PathPlanner::AbstractAction*)),this, SLOT(onPerformActionEM(PathPlanner::AbstractAction*)), Qt::DirectConnection);
        connect(plannerPRM,SIGNAL(sigRestartExplorationPP()),this, SLOT(onRestartExplorationEM()));
        connect(plannerPRM,SIGNAL(sigFrontierToReachPP(Data::Pose)),this, SLOT(onFrontierToReachEM(Data::Pose)));
        ldbg << "Tovar policy"<<endl;
    }
    if(Config::policy==MCDM)
    {
        PRM::PRMAlgorithm* prm= slamModule->getPRM();
        plannerPRM=new PRM::PathPlannerPRM();
        evalFunction= new PRM::PRMFunctionMCDM(prm,plannerPRM,robotId);
        connect(plannerPRM,SIGNAL(sigPerformActionPP(PathPlanner::AbstractAction*)),this, SLOT(onPerformActionEM(PathPlanner::AbstractAction*)), Qt::DirectConnection);
        connect(plannerPRM,SIGNAL(sigRestartExplorationPP()),this, SLOT(onRestartExplorationEM()));
        connect(plannerPRM,SIGNAL(sigFrontierToReachPP(Data::Pose)),this, SLOT(onFrontierToReachEM(Data::Pose)));
        ldbg << "Exploration Module: MCDM policy"<<endl;
    }
}

void ExplorationModule::onPerformActionEM(PathPlanner::AbstractAction* action){
    Action* act = (Action*)action;
    emit sigPerformActionEM(act);
}

void ExplorationModule::onRestartExplorationEM(){
    emit sigRestartExplorationEM();
}


void ExplorationModule::onFrontierToReachEM(Data::Pose pose)
{
    ldbg <<"Exploration: Frontier to reach is " << pose << endl;
    emit sigFrontierToReachEM(pose);

    DestinationMessage *destinationMessage = new DestinationMessage(robotId, pose.position());
    BuddyMessage *buddy = new BuddyMessage(robotNameFromIndex(robotId), destinationMessage);
    Data::WirelessMessage message(buddy, WirelessMessage::MessageExchange);
    coordModule->getCoordinationMessageModule()->onAddNewMessage(message);
}



void ExplorationModule::setSemModule(SemanticMapping::SemanticMappingModule *semModule)
{
    this->semMapModule = semModule;
}

void ExplorationModule::setCoordModule(Coordination::CoordinationModule *coordModule)
{
    this->coordModule = coordModule;
}

void ExplorationModule::setRobotState(Data::RobotState *state)
{
    this->robotState = state;
    ldbg << "Exploration Module: Set robot state. Pose is "<< state->getPose()<<endl;
}

void ExplorationModule::run()
{
    exec();
}

void ExplorationModule::changeStatus(bool activate)
{
    stateMutex->lock();
    active = activate;
    if(activate)
        emit sigStartTimerEM();
    else
        emit sigStopTimerEM();
    //ldbg << "Exploration: State changed. Active? " << activate << endl;
    stateMutex->unlock();
}

void ExplorationModule::onTimeout()
{

    stateMutex->lock();
    bool cont;
    cont = active;
    stateMutex->unlock();

    if(!cont){
        ldbg << "Exploration Module: Should not continue exploration..." << endl;
        return;
    }

    newFrontiersFound = false;

    ldbg << "Exploration Module: Renew map." << endl;

    Map newMap = slamModule->getMap();

    ldbg << "Exploration Module: Search new frontiers." << endl;

    if (Config::OBS::is_test == 0)
        newFrontiersFound = searchNewFrontiers(newMap);
    else
    {
        newMap.frontiers().clear();

        ldbg << "Test_frontier = "<<Config::OBS::test_frontier_x<<", "<<Config::OBS::test_frontier_y<<endl;

        Frontier test_frontier(Config::OBS::test_frontier_x - 0.5, Config::OBS::test_frontier_y -0.5,
                               Config::OBS::test_frontier_x + 0.5, Config::OBS::test_frontier_y +0.5);
        newMap.addFrontier(test_frontier);
        newFrontiersFound = true;
    }

    map = newMap;
    updateSignalStrengths();
}


double ExplorationModule::evaluateFrontier(const Frontier *frontier)
{
    double evaluation;

    ldbg << "Exploration Module: Evaluate Frontier " << frontier << endl;

    QHash<uint, double> *signalPowerData = robotState->getSignalPowerData();
    evaluation = evalFunction->evaluateFrontier(frontier, map, robotState->getBattery(), *signalPowerData);

    ldbg << "Exploration Module: Frontier value is " << evaluation << endl;

    signalPowerData->clear();
    delete signalPowerData;

    return evaluation;
}

EvaluationRecords* ExplorationModule::evaluateFrontiers(const QList<Frontier *> &frontiers)
{
    ldbg << "Exploration Module: Coordination wants me to evaluate some frontiers." << endl;
    return startEvaluation(false, frontiers);
}

bool ExplorationModule::searchNewFrontiers(const SLAM::Map &map)
{
    const QList<Frontier*> oldFrontiers = this->map.frontiers();
    const QList<Frontier*> newFrontiers = map.frontiers();

    if(oldFrontiers.size() != newFrontiers.size())
        return true;

    else if(newFrontiers.size() > 0 && robotState->isIdle()){
        ldbg << "Exploration Module: Robot is idle. Need to evaluate." <<endl;
        return true;
    }
    else
    {
        foreach(Frontier *fn, newFrontiers)
        {
            bool matchFound = false;
            foreach(Frontier *fo, oldFrontiers )
            {
                Point oldCentroid = fo->centroid();
                Point newCentroid = fn->centroid();

                ldbg <<"Exploration: Centroid " << newCentroid << endl;

                if(oldCentroid.distance(newCentroid)>NEW_FRONT_RADIUS){
                    matchFound = true;
                    break;
                }
            }

            if(!matchFound)
            {
                ldbg << "Exploration Module: We have effectively a new frontier!" <<endl;
                return true;
            }
        }
    }
    return false;
}

void ExplorationModule::updateSignalStrengths()
{
    robotState->clearSignalPowerMap();
    WirelessMessage msg(WirelessMessage::SignalQuery);
    msg.setRobotIdForSignalStrength(BASE_STATION_ID);
    emit sigDriverMessageEM(msg);

    uint maxRobot = Config::robotCount;
    for(uint i=0; i<maxRobot; i++){
        if(i!=robotId){
            WirelessMessage msg(WirelessMessage::SignalQuery);
            msg.setRobotIdForSignalStrength(i);
            emit sigDriverMessageEM(msg);
        }
    }
}

void ExplorationModule::setStartingValues()
{
    updateSignalStrengths();
}

void ExplorationModule::onUpdateSignalStrength(const Data::Message &msg)
{
    if(typeid(msg) == typeid(const WirelessMessage &))
    {
        const WirelessMessage &message = (const WirelessMessage &)msg;
        if(message.getCommand() == WirelessMessage::SignalAnswer)
        {
            robotState->insertSignalData(message.getRobotIdForSignalStrength(), message.getSignalPower());
            int robotNum = Config::robotCount;
            if(robotState->getNumberOfPowerDataGathered() == robotNum)
            {
                robotState->setPowerDataGathered();
                emit powerSignalDataGathered();
                stateMutex->lock();
                bool isActive = active;
                stateMutex->unlock();
                if(newFrontiersFound && isActive)
                {
                    forceFrontier = false;
                    const QList<Frontier *> frontiers =  map.frontiers();
                    QString xFrontierIni = "", yFrontierIni = "", xFrontierFin = "", yFrontierFin = "";
                    foreach(Frontier *f, frontiers){
                        xFrontierIni.append(" ").append(QString::number(f->x1(), 'f', 4));
                        xFrontierFin.append(" ").append(QString::number(f->x2(), 'f', 4));
                        yFrontierIni.append(" ").append(QString::number(f->y1(), 'f', 4));
                        yFrontierFin.append(" ").append(QString::number(f->y2(), 'f', 4));
                    }


                    ldbg << "Exploration Module: Found new frontiers!" << endl;
                    ldbg << "xFront=[" << xFrontierIni << "; "<< xFrontierFin <<"];" << endl;
                    ldbg << "yFront=[" << yFrontierIni << "; "<< yFrontierFin <<"];" << endl;

                    ldbg << "Start a new evaluation." << endl;

                    startEvaluation(true, frontiers);
                }
            }
        }
    }
}

EvaluationRecords *ExplorationModule::startEvaluation(bool startNewAuction, const QList<Frontier *> &frontiers)
{
    ldbg << "Exploration Module: Start evaluation of frontiers" << endl;
    ldbg << "Exploration Module: I have " << frontiers.size() << " theorical frontiers to evaluate" << endl;

    QList<SLAM::Geometry::Frontier *> frontToUse,tempFrontToUse;
    foreach(Frontier *frontier, frontiers){
        ldbg << frontier->centroid() << endl;
    }
    if(badFrontiersCentroids->size() > 0)
    {
        tempFrontToUse = removeBadFrontiers(frontiers);
        ldbg << "Exploration Module: I have " << tempFrontToUse.size() << " real frontiers to evaluate" << endl;

        foreach(Frontier *frontier, tempFrontToUse){
            ldbg << frontier->centroid() << endl;
        }
        forceFrontier = true;
    }
    else
    {
        tempFrontToUse = frontiers;
    }
    frontToUse = filterFrontiers(tempFrontToUse);

    if(frontToUse.size() == 0)
    {
        //emit a signal to go back of .5
        ldbg << "Exploration Module: No new frontiers!! Go back and  restart!" << endl;
        emit sigNoFrontierAvailableEM();
        cleanBadFrontiers();
        EvaluationRecords *evalRec = new EvaluationRecords();
        foreach(Frontier *f, frontiers){
            evalRec->putEvaluation(*f, 0.0);
        }

        return evalRec;
    }

    QHash<uint, double> *signalPowerData = robotState->getSignalPowerData();

    bool allNan = false;
    EvaluationRecords * evalRec = NULL;
    if(allNan){
        ldbg<<"Robot Isolated"<<endl;
    } else {
        int robotBattery = robotState->getBattery();
        ldbg << "Exploration Module: Call evaluation function." << endl;

        emit sigStopRobotForPlanningEM();

        evalRec = evalFunction->evaluateFrontiers(frontToUse, map, robotBattery, *signalPowerData);

        ldbg << "Exploration Module: frontiers evaluated!" << endl;

        stateMutex->lock();
        bool shouldContinue = active;
        stateMutex->unlock();

        if(startNewAuction && shouldContinue)
        {
            //call coordination part, passing evalRec.
            ldbg << "Exploration Module: trying to start a new auction" << endl;
            coordModule->startNewFrontierAuction(evalRec, robotState->isIdle());
        } else if(forceFrontier){
            ldbg << "Exploration Module: force assignment for bad path" << endl;
            coordModule->startNewFrontierAuction(evalRec, true);
        }

    }
    signalPowerData->clear();
    delete signalPowerData;
    return evalRec;
}

bool ExplorationModule::checkIfAllNan(QHash<uint, double> *signalPowerData)
{
    foreach(uint k, signalPowerData->keys()){
        if(!isnan(signalPowerData->value(k))){
            return false;
        }
    }
    return true;
}

void ExplorationModule::onEnableUserCriterionSignal(bool activate, const Data::HighLevelCommand *command)
{
    evalFunction->onEnableUserCriteria(activate, command);
}

void ExplorationModule::handleBadFrontier(const Data::Pose pose){
    ldbg << "Exploration Module: adding a bad frontiers" << endl;
    //Append the bad point to the list
    Point *badPoint = new Point(pose.x(), pose.y());
    badFrontiersCentroids->append(badPoint);

    //  get the frontiers from the map.
    QList<Frontier *> frontiers = map.frontiers();
    qDebug("%d %d",frontiers.size(), badFrontiersCentroids->size());
    if (frontiers.size() == badFrontiersCentroids->size())
    {
        ldbg <<"Exploration Module: No frontiers available. Go back."<<endl;
        emit sigNoFrontierAvailableEM();
    }

    changeStatus(true);
}

const QList<SLAM::Geometry::Frontier *> ExplorationModule::removeBadFrontiers(const QList<SLAM::Geometry::Frontier* > &frontiers)
{
    bool isBadFrontier;
    QList<SLAM::Geometry::Frontier *> toRet;
    ldbg << "Exploration Module: Frontiers size BEFORE bad frontiers check = " << frontiers.size() << endl;
    for(int i=0; i<frontiers.size(); i++){
        //search the bad frontiers from the one i get
        isBadFrontier = false;
        for(int j=0; j<badFrontiersCentroids->size(); j++){
            if(frontiers[i]->centroid().distance(*(badFrontiersCentroids->at(j))) < NEW_FRONT_RADIUS){
                ldbg << "Exploration Module: la frontiera cattiva �: " << frontiers[i]->centroid() << endl;
                ldbg << "Exploration Module: il bad frontier che triggera � :" << badFrontiersCentroids->at(j) << endl;
                isBadFrontier = true;
                break;
            }
        }
        //remove the bad frontiers
        if(!isBadFrontier){
            ldbg << "Exploration Module: Remove frontier at : " << frontiers[i]->centroid() << endl;
            toRet.append(frontiers.at(i));
        }
    }
    ldbg << "Exploration Module: Frontiers size AFTER bad frontiers check = " << toRet.size() << endl;
    return toRet;
}

void ExplorationModule::cleanBadFrontiers()
{
    ldbg << "Exploration Module: Cleaning bad frontiers" << endl;
    //Clean the list
    while(!badFrontiersCentroids->isEmpty()){
        delete badFrontiersCentroids->last();
        badFrontiersCentroids->removeLast();
    }
    //Restart the timer
    emit sigStartTimerEM();
}

void ExplorationModule::onStartTimer()
{
    mapRenewTimer->start(MAP_RENEW_TIME);
}

void ExplorationModule::onStopTimer()
{
    mapRenewTimer->stop();
}

QList<SLAM::Geometry::Frontier *> ExplorationModule::filterFrontiers(QList<Frontier *> frontiers){
    QList<SLAM::Geometry::Frontier *> newFrontiers;
    foreach(Frontier* f,frontiers){
        uint owner=slamModule->getFrontierOwner(*f);
        if(owner==robotId){
            newFrontiers.append(f);
        }
    }
    return newFrontiers;
}

}
