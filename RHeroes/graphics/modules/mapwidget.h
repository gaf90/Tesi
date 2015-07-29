#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGraphicsView>
#include "data/pose.h"
#include "baseStation/graphicparams.h"
#include "graphics/utils/robotgraphicitem.h"
#include "slam/geometry/linesegment.h"
#include "slam/geometry/frontier.h"
#include "graphics/utils/mapgraphicsscene.h"
#include "graphics/utils/waypointmarkergraphicsitem.h"
#include "graphics/utils/victimgraphicitem.h"
#include "graphics/utils/customlabelgraphicitem.h"
#include "graphics/utils/destinationgraphicitem.h"
#include "graphics/utils/popupwidget.h"
#include "graphics/utils/exploreareagraphicitem.h"
#include "graphics/utils/exploredirectiongraphicitem.h"
#include "graphics/utils/notificationgraphicsitem.h"
#include "graphics/utils/dangergraphicsitem.h"

#define MM_NOTIFICATION_TIME 3000

namespace graphics{

/**
* @brief this class manages the map viewer
*
* The MapWidget class is used to visualize the global map created by the system
* and manage input commands from the human operator.
*
* @see MapManager
*/
class MapWidget : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = 0);

    virtual ~MapWidget();

    void handleKeyPressEvent(QKeyEvent *e);
    void handleKeyReleaseEvent(QKeyEvent *e);

    void centerOnRobot(uint robotID);
    void centerOnPose(Data::Pose pose);

    void tempNotificationMarker(qreal x, qreal y);

//--Methods for graphical components management---------------------------------------
    /**
    * This method is used to manage the switching process to the Teleoperation view.
    */
    void switchToTeleoperationView();

    /**
    * This method is used to manage the switching process to the Global view.
    */
    void switchToGlobalView();

    /*Provvisorio, metodo per aggiornare la mappa a video. Vengono stampate tutte le linee
    ricevute come parametro*/
    void drawMap(QList<SLAM::Geometry::LineSegment *> lines, uint robotID, bool affidable);

    //! method used for drawing frontiers. This is used only for debug purposes.
    void drawFrontiers(QList<SLAM::Geometry::Frontier *> frontiers, uint robotID);

    void drawPath(QList<SLAM::PathNode *> poses, uint robot);

    /*Provvisorio: aggiunge dell'informazione semantica alla mappa, in corrispondenza delle
    coordinate nella Pose. Eventualmente si sostituisce la stringa con un'icona, durante il
    metodo. Da gestirsi i mapping stringa-icona eventuali.*/
    void addSemanticInfoToMap(Data::Pose coordinates, QString info);

    /*Mostra la posizione del robot, orientata, nella mappa. Si disegna il robot in
    modo diverso se selezionato.*/
    void showRobot(Data::Pose place, bool selected, uint robotID);

    /**
    * This method shows a victim on the map at the given position.
    */
    void showVictim(Data::Pose place, uint victimID);

    /*Pulisce la mappa completamente per poterne visualizzare le eventuali versioni successive*/
    void cleanMap();

    /*Pulisce la mappa dalla posizione dei robot. Da valutare l'effettiva utilità della cosa...*/
    void cleanRobots();

    /**
    * this method deletes a victim from the map.
    */
    void deleteVictim(uint id);

    //! Handles the change of the selected robot, to make the map consistent with the global status.
    void onSelectedRobotChanged(uint id);

    //! Method used to mark a robot as unreachable
    void setRobotUnreachable(uint id);

    //! Method used to mark a robot as reachable again
    void setRobotReachable(uint id);

    //! Shows the robot next destination as established by coordination module
    void setRobotDestination(uint robotID, Data::Pose position);

    //! Shows a popup frame on the map for information visualization
    void showPopup(int id, Data::Pose position, QString module, QString message);

signals:
    void signalDirectionHighLevelCommand(double x, double y, uint robotID);
    void signalAreaHighLevelCommand(double x, double y, uint robotID);

    void signalAddWaypoint(Data::Pose coordinates, uint robotID);
    void signalSendWaypointCommand();
    void signalRobotSelected(uint robotID);
    void signalVictimSelected(uint victimID);
    void signalVictimMoved(uint victimID, Data::Pose position);
    void signalVictimAdded(Data::Pose position);

    void signalPopupMessageArchived(int id, QString module, QString result);

public slots:
    //! This slot is used to save the map content into an SVG file.
    void saveSVG();

    //! This slot saves the map to a txt file for further elaboration
    void saveForMods();

    //! thsi slot must be invoked when a waypoint is reached by a robot. It provides user notification
    //! on the map and deletes the draw of the waypoint.
    void onWaypointReached(uint robot);

//--Slots for the management of operator's imputs----------------------------------------


//--Slots for graphical changes driven by the user---------------------------------------

    //! Slot that handles the selection of a robot by the user on the map.
    void onRobotSelected(uint id);

    //! Slot that handles the selection of a victim by the user.
    void onVictimSelected(uint id);

    void onMessageRemoval(int id);

    void onNotificationTimeExpired();

private slots:
    void onWaypointSetted(Data::Pose pose);
    void onVictimMoved(uint id,const Data::Pose &position);
    void onCustomLabelAdded(Data::Pose pos, QString mex);
    void onDangerLabelAdded(Data::Pose pos);
    void onVictimAddedByOperator(Data::Pose position);
    void onMessageInteraction(int messageId, QString module, QString result);

    void onExploreDirection(double x, double y);
    void onExploreArea(double x, double y);

protected:
    /**
    * This functions allow the user to drag and drop the VictimGraphicItem in order to change its
    * position on the map,
    */
    void dragEnterEvent(QGraphicsSceneDragDropEvent *event);
    void dragLeaveEvent(QGraphicsSceneDragDropEvent *event);
    void dropEvent(QDropEvent *event);
    void dragMoveEvent(QDragMoveEvent *event);

private:
    void setupUi();

    void wheelEvent(QWheelEvent *event);

    /*Il metodo permette di rimpicciolire la mappa tenendo come punto di riferimento centro.*/
    void zoomOut();

    /*Il metodo permette di ingrandire la mappa sul punto di coordinate centro.*/
    void zoomIn(QPoint center);

    void cleanActions(uint robot);

    void deleteWalls(QList<QGraphicsLineItem*> *walls);

    void cleanRobotPaths();

    QHash<uint, RobotGraphicItem*> *robots;
    QHash<uint, VictimGraphicItem*> *victims;
    QHash<uint, WaypointMarkerGraphicsItem* > *waypoints;
    QHash<uint, DestinationGraphicItem* > *destinations;
    QHash<uint, ExploreDirectionGraphicItem*> *explorationDirections;
    QHash<uint, ExploreAreaGraphicItem*> *explorationAreas;

    QList<SLAM::Geometry::LineSegment*> *mappa;
    QHash<uint,QList<QGraphicsLineItem*>* > *walls;
    QHash<uint,QList<QGraphicsLineItem*>* > *frontiers;
    QHash<uint, QList<QGraphicsEllipseItem*>* > *robotPaths;
    QList<CustomLabelGraphicItem*> *customLabels;
    QList<DangerGraphicsItem*> *dangerLabels;
    QList<PopupWidget*> *popupMessages;
    MapGraphicsScene *scene;

    QWheelEvent *wheel;

    uint selectedRobot;
    uint selectedVictim;

    double actualZoom;

    QTimer *notificationTimer;
    QList<NotificationGraphicsItem*> notificationMarkers;

    QList<uint> *kenafs;
};


}
#endif // MAPWIDGET_H
