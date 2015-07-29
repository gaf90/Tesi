#ifndef MAPMANAGER_H
#define MAPMANAGER_H

#include <QObject>
#include "data/waypointcommand.h"
#include "data/highlevelcommand.h"
#include "data/mapmessage.h"
#include "data/semanticmapinfomessage.h"
#include "data/pose.h"
#include "graphics/modules/mapwidget.h"

namespace BaseStation{

/**
* @brief manager of the map in every interface
*
* The MapManager class is used to manage every map element coming from sigle robots,
* integrate them and visualize a global map to the user.
* It allows to visualize the more recent map, robot positions (highlighting selected ones),
* and robot's paths (or waypoints). It allows too to assign high level commands and waypoint
* commands to both single robots and group of robots.
*
* @see BaseStation
*
* @author Alain Caltieri
*/
class MapManager : public QObject
{
    Q_OBJECT
public:
    /**
    * Create the MapManager object to be used by the BaseStation to manage every event
    * and operation related to maps.
    */
    explicit MapManager(QObject *parent = 0, QList<uint> *kenafs = 0);
    
    virtual ~MapManager();

    /**
    * This method is used to retrieve the User interface related to the MapManager class
    * so that it can be used in a composed interface. No Information Hiding provided, but
    * MapManager still remains the only way to interact with the mapViewer
    * @return the reference to the graphics::MapWidget element that manages the map's UI
    */
    graphics::MapWidget* getUi();

    void addKenaf(uint id);

signals:
    void sigWaypointCommand(Data::Pose waypoint, bool notifyWhenFinished, uint waitTime, uint robotID);
    void sigHighLevelCommandMM(double x, double y, bool notifyWhenFinished, uint waitTime, uint robotID, bool isDirection);
    void sigSelectedRobotChangedMM(uint robotID);
    void signalSemanticInfoAdded(/*TODO*/);
    void signalVictimSelectedChanged(uint id);
    void signalVictimMoved(uint victimID, Data::Pose position);
    void signalVictimAddedByUser(Data::Pose position);
    void signalNotificationHandled(int id, QString module, QString result);

public slots:

//-----------New Info Management---------------------------------------------------------------
    /**
    * This slot is used to handle new data regarding maps.
    * @param map the Data::MapMessage message containing last map, extimated map and
    * robot positions.
    */
    void onNewMapDataReceived(const SLAM::Map &newMap, uint robotID);

//    /**
//    * The slot is used to handle incoming semantic informations to be showed (evenctually)
//    * on the map
//    * @param info the Data::SemanticMapInfoMessage message used to transfer semantic information
//    * between single robots and base station.
//    */
//    void onNewSemanticInfo(Data::SemanticMapInfoMessage info);

    /**
    * This slot handles the event waypoint reached
    * @param robotID the id of the involved robot.
    */
    void onWaypointReached(uint robotID);

    void onFaultDetected(QString, uint robotID);

    void onNewDestinationReceived(uint robotID, Data::Pose position);

    /**
    * Handling of a new victim. It will be added to the map.
    * @param position the Data::Pose of the victim
    * @param victimID the uint that identifies the victim
    */
    void onVictimFound(Data::Pose position, uint victimID);

    /**
    * Deletes the victim from the map
    * @param victimID the id of the deleted victim
    */
    void onVictimDeleted(uint victimID);

    /**
    * This slot handles a message popup that must be shown on the map.
    */
    void onMessagePopup(int id, Data::Pose position, QString module, QString message);

//-----------Interface changes management------------------------------------------------------
//    /**
//    * This method manages the event selected robots changed, that can be triggered by
//    * many modules (MapManager included).
//    *
//    * @param robotNames QList<QString> with the current selected robot's names.
//    */
//    void onSelectedRobotChangedMM(QList<QString> robotNames);

    /**
    * This method manages the event selected robots changed, that can be triggered by
    * many modules (MapManager included).
    *
    * @param robotID uint with the current selected robot's id.
    */
    void onSelectedRobotChangedMM(uint robotID);

    /**
    * This slot handles the selection of a robot in the mapViewer
    * @param robotID the id of the robot selected
    */
    void onRobotSelected(uint robotID);

    /**
    * This slot handles the selection of a victim
    * @param id the uint that identifies the victim
    */
    void onVictimSelected(uint id);

    /**
    * This slot handles the selection of a victim through the map widget
    * @param id the uint that identifies the victim
    */
    void onVictimSelectedChanged(uint id);

    /**
    * This method is used to manage the switching process to the Teleoperation UI.
    */
    void onSwitchToTeleoperation();

    /**
    * This method is used to manage the switching process to the Global UI.
    */
    void onSwitchToGlobal();

    /**
    * This slot is used to handle the marking of a robot as unreachable
    */
    void onRobotUnreachable(uint id);

    /**
    * This slot is used to handle the marking of a robot as reachable again
    */
    void onRobotReachableAgain(uint id);

//----------Commands management-----------------------------------------------------------------
    /**
    * This slot manages the creation of a direction high-level command to be sent to a single robot.
    */
    void onDirectionHighLevelCommand(double x, double y, uint robotID);
    /**
    * This slot manages the creation of a area high-level command to be sent to a single robot.
    */
    void onAreaHighLevelCommand(double x, double y, uint robotID);

//----------Miscelaneous--------------------------------------------------------------------------
    void onFocusOnPoint(const Data::Pose &pose);
    void onFocusOnRobot(uint robotID);

    void changeStatus(uint robotID);

private slots:
    void onWaypointCommand(Data::Pose pose, uint robotID);
    void onVictimMoved(uint id,const Data::Pose &position);
    void onVictimAddedByUser(Data::Pose position);
    void onNotificationArchived(int id, QString module, QString result);

private:
    graphics::MapWidget *mapViewer;
    QList<Data::Pose*> *tempWaypoints;

    uint selectedRobot;
    //! Used for debugging until Mazy doesn't finish the map integration module
    QHash<uint, QList<SLAM::Geometry::LineSegment*> > tempHistory;
    QHash<uint, QList<SLAM::Geometry::Frontier*> > tempfrontiers;
    QHash<uint, Data::Pose> tempPoses;

    QList<uint> *kenafList;
    QList<uint> *simpleMaps;
};

}

#endif // MAPMANAGER_H
