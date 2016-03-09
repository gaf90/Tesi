/*
 * slammodule.cpp
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#include "slammodule.h"
#include "slam/engine/deterministicslam.h"
#include "slam/legacy/iclslam.h"
#include "data/wirelessmessage.h"
#include <typeinfo>

namespace SLAM {

using namespace Data;
using namespace Geometry;
using namespace Legacy;

SLAMModule::SLAMModule(uint robotId, const Data::Pose &initialPose) :
    QThread(),
    robotId(robotId),
    //PRM
    prm(new PRM::PRMAlgorithm::PRMAlgorithm()),
    firstIteration(true)
  //
{
    slam = new SLAM::Engine::DeterministicSLAM(robotId, initialPose);
#ifndef SLAM_SKIP_DEBUG
    ldbg << "Initial Pose = " << initialPose << endl;
#endif

    moveToThread(this);
    connect(this, SIGNAL(scanReceived()),     this, SLOT(handleScan()),     Qt::QueuedConnection);
    connect(this, SIGNAL(odometryReceived()), this, SLOT(handleOdometry()), Qt::QueuedConnection);
    connect(this, SIGNAL(insReceived()),      this, SLOT(handleINS()),      Qt::QueuedConnection);
    connect(this, SIGNAL(mapKOReceived()),    this, SLOT(handleMapKO()),    Qt::QueuedConnection);
    connect(this, SIGNAL(outdoorReceived()),  this, SLOT(handleOutdoor()),  Qt::QueuedConnection);
    //PRM
    qRegisterMetaType<SLAM::Map>("Map");
    connect(this, SIGNAL(newMapPRM(Map)),  prm, SLOT(handleNewMap(Map)),  Qt::QueuedConnection);
    //
    connect(slam, SIGNAL(newRobotPose(TimedPose)), this,
            SLOT(forwardRobotPose(TimedPose)), Qt::DirectConnection);
    start();
}

SLAMModule::~SLAMModule()
{
    delete slam;
}

Map SLAMModule::getMap(){
    return getMap(false);
}

uint SLAMModule::getFrontierOwner(Frontier frontier){
    foreach(Map* m,robotsMap){
        if(robotsMap.key(m)<robotId){
            foreach(Frontier* f,m->frontiers()){
                if(*f==frontier){
                    return robotsMap.key(m);
                }
            }
        }
    }
    return robotId;
}

void SLAMModule::updateMap(uint mapId){
    mutex.lock();
    ((SLAM::Engine::DeterministicSLAM*)slam)->mergeMap(*robotsMap[mapId],mapId);
    mutex.unlock();

}

Map SLAMModule::getMap(bool fullMap)
{
    mutex.lock();
    Map ret = ((SLAM::Engine::DeterministicSLAM*)slam)->getMap(fullMap);
    foreach(Map* otherMap,robotsMap){
        uint otherMapId=robotsMap.key(otherMap);
        const SLAM::TimedPose* p=otherMap->lastRobotPose(otherMapId);
        if((p!=NULL)&&(otherMapId!=robotId)){
            ret.addPose(otherMapId,*p);
        }
    }
    mutex.unlock();
    return ret;
}

void SLAMModule::onSensorData(const Message &msg)
{
    if(typeid(msg) == typeid(const LaserData &)) {
        const LaserData &scan = (const LaserData &) msg;
        TimedPointScan tps = {scan.getTimestamp(), new PointScan(scan)};
        scanQueue.enqueue(tps);
        emit scanReceived();
    } else if(typeid(msg) == typeid(const OdometryData &)) {
        const OdometryData &odo = (const OdometryData &) msg;
        odoQueue.enqueue(new OdometryData(odo));
        emit odometryReceived();
    } else if(typeid(msg) == typeid(const INSData &)) {
        const INSData &ins = (const INSData &) msg;
        insQueue.enqueue(new INSData(ins));

        emit insReceived();
    } else if(typeid(msg) == typeid(const WirelessMessage &)) {
        const WirelessMessage &wifi = (const WirelessMessage &) msg;

        if (wifi.getCommand() == WirelessMessage::MessageExchange
                && wifi.getBuddyMessage()->getContent() == BuddyMessage::MapInformation){
            //qDebug("MERGED!");
            SLAM::Map *map = new SLAM::Map(*wifi.getBuddyMessage()->get<SLAM::Map>());

            uint id=robotIndexFromName(wifi.getBuddyMessage()->getSource());

            robotsMap[id]=map;
            updateMap(id);
        }

        if(wifi.getCommand() == WirelessMessage::MessageExchange &&
                wifi.getBuddyMessage()->getContent() == BuddyMessage::ErrorNotification &&
                wifi.getBuddyMessage()->get<ErrorNotificationMessage>()->getInvolvedModule() ==
                ErrorNotificationMessage::MapKO) {
            emit mapKOReceived();
        } else if(wifi.getCommand() == WirelessMessage::MessageExchange &&
                  wifi.getBuddyMessage()->getContent() == BuddyMessage::ModuleActivation &&
                  wifi.getBuddyMessage()->get<ModuleActivationMessage>()->getInvolvedModule() ==
                  QString(POARET_SLAM_MODULE) &&
                  !wifi.getBuddyMessage()->get<ModuleActivationMessage>()->getModuleStatus()) {
            emit outdoorReceived();
        }
    }
}

void SLAMModule::handleScan()
{
    mutex.lock();

    TimedPointScan tps = scanQueue.dequeue();
    slam->handleScan(tps.timestamp, *(tps.scan));
    delete tps.scan;

    mutex.unlock();
}

void SLAMModule::handleOdometry()
{
    mutex.lock();

    OdometryData *odo = odoQueue.dequeue();
    slam->handleOdometry(odo->getTimestamp(), odo->getPose());
    delete odo;

    mutex.unlock();
}

void SLAMModule::handleINS()
{
    mutex.lock();

    INSData *ins = insQueue.dequeue();
    slam->handleINS(*ins);
    delete ins;

    mutex.unlock();
}

void SLAMModule::handleMapKO()
{
    mutex.lock();

    /*
        TODO: currently only ICLSLAM implements these, handle them better with class
        abstraction in the future
    */
    SLAM::Engine::DeterministicSLAM* deterministicSlam =  dynamic_cast<SLAM::Engine::DeterministicSLAM *>(slam);
    //ICLSLAM *iclSLAM = dynamic_cast<ICLSLAM *>(slam);
    //if(iclSLAM) iclSLAM->handleMapKO();
    if(deterministicSlam){
        deterministicSlam->handleMapKO();
    }
    mutex.unlock();
}

void SLAMModule::handleOutdoor()
{
    mutex.lock();

    /*
        TODO: currently only ICLSLAM implements these, handle them better with class
        abstraction in the future
    */
    SLAM::Engine::DeterministicSLAM* deterministicSlam =  dynamic_cast<SLAM::Engine::DeterministicSLAM *>(slam);
    //ICLSLAM *iclSLAM = dynamic_cast<ICLSLAM *>(slam);
    //if(iclSLAM) iclSLAM->handleOutdoor();
    if(deterministicSlam){
        deterministicSlam->handleOutdoor();
    }
    mutex.unlock();
}

void SLAMModule::forwardRobotPose(TimedPose pose)
{
    emit newRobotPose(pose);
    //PRM
    //if(Config::policy==Exploration::PRM){
        //ldbg << "PRM Algorithm: get map" << endl;
        Map map = ((SLAM::Engine::DeterministicSLAM*)slam)->getMap(false);
        if(map.getMaxX()!=-INFINITY){
            //ldbg << "PRM Algorithm: update PRM" << endl;
            //prm->updatePRM(map);
            if(firstIteration){
                //ldbg<<"Sending map first iteration"<<endl;
                emit newMapPRM(map);
                firstIteration=false;
            }
            else{
                const Pose* newPose = map.lastRobotPose(Config::robotID);
                Pose oldPose = prm->getOldRobotPose();
                if(newPose->getDistance(oldPose)>Config::PRM::movementRadius/2){
                    //ldbg<<"sending map!!!!!!!!!"<<endl;
                    emit newMapPRM(map);
                }
            }
            //            emit newMapPRM(map);
            //ldbg << "PRM Algorithm: inviato segnale new map" << endl;
        }
    //}
    //ldbg << "inviato segnale new robot pose" << endl;
    //
}

void SLAMModule::run()
{
    exec();
}

//PRM
PRM::PRMAlgorithm* SLAMModule::getPRM(){
    return prm;
}
//
} /* namespace SLAM */
