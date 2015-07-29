#include "mapmanager.h"
#include <QHash>

using namespace Data;
using namespace graphics;
using namespace SLAM;
namespace BaseStation{

MapManager::MapManager(QObject *parent, QList<uint> *kenafs) :
    QObject(parent),
    mapViewer(new MapWidget()),
    tempWaypoints(new QList<Pose*>()),
    selectedRobot(999), tempHistory(QHash<uint, QList<Geometry::LineSegment*> >()),
    tempfrontiers(QHash<uint, QList<Geometry::Frontier*> >()), tempPoses(QHash<uint, Data::Pose>()),
    simpleMaps(new QList<uint>())
{
    connect(mapViewer, SIGNAL(signalRobotSelected(uint)), this, SLOT(onRobotSelected(uint)));
    connect(mapViewer, SIGNAL(signalAddWaypoint(Data::Pose,uint)), this, SLOT(onWaypointCommand(Data::Pose,uint)));
    connect(mapViewer, SIGNAL(signalVictimSelected(uint)), this, SLOT(onVictimSelectedChanged(uint)));
    connect(mapViewer, SIGNAL(signalVictimMoved(uint,Data::Pose)), this, SLOT(onVictimMoved(uint,Data::Pose)));
    connect(mapViewer, SIGNAL(signalVictimAdded(Data::Pose)), this, SLOT(onVictimAddedByUser(Data::Pose)));
    connect(mapViewer, SIGNAL(signalAreaHighLevelCommand(double,double,uint)),
            this,SLOT(onAreaHighLevelCommand(double,double,uint)));
    connect(mapViewer, SIGNAL(signalDirectionHighLevelCommand(double,double,uint)),
            this, SLOT(onDirectionHighLevelCommand(double,double,uint)));
    if(kenafs != 0)
        kenafList = new QList<uint>(*kenafs);
    else
        kenafList = new QList<uint>();
}

MapManager::~MapManager()
{
    while(!this->tempWaypoints->isEmpty())
        delete tempWaypoints->takeFirst();
    delete tempWaypoints;
}

void MapManager::onNewMapDataReceived(const SLAM::Map &newMap, uint robotID)
{
    mapViewer->drawPath(newMap.robotPath(robotID), robotID);
//    if(simpleMaps->contains(robotID))
//    {
//    }
    if(!kenafList->contains(robotID) && !simpleMaps->contains(robotID))
    {
        mapViewer->drawMap(newMap.walls(), robotID, true);
        mapViewer->drawFrontiers(newMap.frontiers(), robotID);
    }
    else
    {
        mapViewer->drawMap(newMap.walls(), robotID, true);
        mapViewer->drawFrontiers(newMap.frontiers(), robotID);
    }
    mapViewer->showRobot(*newMap.lastRobotPose(robotID),false, robotID);
}

//void MapManager::onNewSemanticInfo(SemanticMapInfoMessage)
//{
//}

//void MapManager::onSelectedRobotChangedMM(QList<QString> robotNames)
//{
//}

void MapManager::onSelectedRobotChangedMM(uint robotID)
{
    this->selectedRobot = robotID;
    mapViewer->onSelectedRobotChanged(robotID);
//    mapViewer->onVictimSelected(999);
}

void MapManager::onRobotSelected(uint robotID)
{
    if (robotID != selectedRobot)
    {
        selectedRobot = robotID;
        emit sigSelectedRobotChangedMM(robotID);
    }
//    mapViewer->onVictimSelected(999);
}

void MapManager::onVictimSelected(uint id)
{
    mapViewer->onVictimSelected(id);
}

void MapManager::onVictimSelectedChanged(uint id)
{
    emit signalVictimSelectedChanged(id);
}

void MapManager::onSwitchToTeleoperation()
{
}

void MapManager::onSwitchToGlobal()
{
}

void MapManager::onRobotUnreachable(uint id)
{
    mapViewer->setRobotUnreachable(id);
}

void MapManager::onRobotReachableAgain(uint id)
{
    mapViewer->setRobotReachable(id);
}

void MapManager::onDirectionHighLevelCommand(double x, double y, uint robotID)
{
    emit sigHighLevelCommandMM(x, y, true, 5000, robotID, true);
}

MapWidget* MapManager::getUi()
{
    return this->mapViewer;
}

void MapManager::addKenaf(uint id)
{
    if(!kenafList->contains(id))
        kenafList->append(id);
}

void MapManager::onWaypointCommand(Data::Pose pose, uint robotID)
{
//    ldbg << "map manager: " << pose.getX() <<"," << pose.getY() << endl;
    emit sigWaypointCommand(pose,true, 10000, robotID);
}

void MapManager::onVictimMoved(uint id, const Pose &position)
{
    emit signalVictimMoved(id, position);
}

void MapManager::onVictimAddedByUser(Pose position)
{
    emit signalVictimAddedByUser(position);
}

void MapManager::onNotificationArchived(int id, QString module, QString result)
{
    emit signalNotificationHandled(id, module, result);
}

void MapManager::onWaypointReached(uint robotID)
{
    mapViewer->onWaypointReached(robotID);
}

void MapManager::onFaultDetected(QString, uint robotID)
{
    mapViewer->onWaypointReached(robotID);
}

void MapManager::onNewDestinationReceived(uint robotID, Pose position)
{
    mapViewer->setRobotDestination(robotID,position);
}

void MapManager::onVictimFound(Pose position, uint victimID)
{
    mapViewer->showVictim(position, victimID);
}

void MapManager::onVictimDeleted(uint victimID)
{
    mapViewer->deleteVictim(victimID);
}

void MapManager::onMessagePopup(int id, Data::Pose position, QString module, QString message)
{
    mapViewer->showPopup(id, position, module, message);
}

void MapManager::onAreaHighLevelCommand(double x, double y, uint robotID)
{
    emit sigHighLevelCommandMM(x, y, true, 5000, robotID, false);
}

void MapManager::onFocusOnPoint(const Pose &pose)
{
    mapViewer->centerOnPose(pose);
}

void MapManager::onFocusOnRobot(uint robotID)
{
    mapViewer->centerOnRobot(robotID);
}

void MapManager::changeStatus(uint robotID)
{
    if(!simpleMaps->contains(robotID))
        simpleMaps->append(robotID);
}


}

