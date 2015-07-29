#include "mapwidget.h"
#include <QBoxLayout>
#include <QWheelEvent>
#include <typeinfo>
#include <QHash>
#include <QtSvg/QSvgGenerator>
#include <QFileDialog>
#include <QPainter>
#include <QTextStream>
#include <QDrag>
#include <QCursor>
#include <QApplication>
#include <QMimeData>
#include <QMessageBox>
#include <QGraphicsSceneDragDropEvent>
#include "math.h"
#include "slam/geometry/rototranslation.h"
#include <QGraphicsProxyWidget>

#define NO_VICTIM_SELECTED 999

using namespace SLAM::Geometry;
namespace graphics{

MapWidget::MapWidget(QWidget *parent):
    robots(new QHash<uint, RobotGraphicItem*>()), victims(new QHash<uint, VictimGraphicItem*>()),
    waypoints(new QHash<uint, WaypointMarkerGraphicsItem*>()),
    destinations(new QHash<uint, DestinationGraphicItem*>()),
    explorationDirections(new QHash<uint, ExploreDirectionGraphicItem*>()),
    explorationAreas(new QHash<uint, ExploreAreaGraphicItem*>()),
    mappa(new QList<LineSegment*>()), walls(new QHash<uint,QList<QGraphicsLineItem*>* >()),
    frontiers(new QHash<uint,QList<QGraphicsLineItem*>* >()), robotPaths(new QHash<uint, QList<QGraphicsEllipseItem*>* >()),
    customLabels(new QList<CustomLabelGraphicItem*>()),
    dangerLabels(new QList<DangerGraphicsItem*>()),
    popupMessages(new QList<PopupWidget*>()),
    selectedRobot(999), selectedVictim(NO_VICTIM_SELECTED),
    actualZoom(1), notificationTimer(new QTimer()), kenafs(new QList<uint>())
{
    Q_UNUSED(parent);
    this->scene = new MapGraphicsScene();
    this->setMaximumHeight(MAP_WIDGET_HEIGHT_GLOBAL);
    this->setMaximumWidth(MAP_WIDGET_WIDTH_GLOBAL);
    this->setScene(scene);
    this->setupUi();
    connect(scene, SIGNAL(signalWaypoint(Data::Pose)), this, SLOT(onWaypointSetted(Data::Pose)));
    connect(scene, SIGNAL(signalLabelAdded(Data::Pose,QString)), this, SLOT(onCustomLabelAdded(Data::Pose,QString)));
    connect(scene, SIGNAL(signalDangerLabelAdded(Data::Pose)), this, SLOT(onDangerLabelAdded(Data::Pose)));
    connect(scene, SIGNAL(signalVictimAddedByUser(Data::Pose)), this, SLOT(onVictimAddedByOperator(Data::Pose)));
    connect(scene, SIGNAL(signalExploreDirection(double,double)), this, SLOT(onExploreDirection(double,double)));
    connect(scene, SIGNAL(signalExploreArea(double,double)), this, SLOT(onExploreArea(double,double)));
    setCursor(Qt::CrossCursor);
    setAcceptDrops(true);
    //testing:
//    this->showPopup(1, Data::Pose(0,0,0), "Testing banana", "Oh tropicanaaaaa.....");
//    this->showPopup(2, Data::Pose(10,10,10), "Testing banana 2222", "Oh tropicanaaaaa.....");
}

MapWidget::~MapWidget()
{
}

void MapWidget::handleKeyPressEvent(QKeyEvent *e)
{
    this->keyPressEvent(e);
}

void MapWidget::handleKeyReleaseEvent(QKeyEvent *e)
{
    this->keyReleaseEvent(e);
}

void MapWidget::centerOnRobot(uint robotID)
{
    QGraphicsItem * r = robots->value(robotID, 0);
    if(r != 0)
    {
        this->centerOn(r);
        tempNotificationMarker(r->x(), r->y());
    }
}

void MapWidget::centerOnPose(Data::Pose pose)
{
    this->centerOn(pose.getX()*SLAM_SCALE_FACTOR, pose.getY()*SLAM_SCALE_FACTOR);
    tempNotificationMarker(pose.getX()*SLAM_SCALE_FACTOR, pose.getY()*SLAM_SCALE_FACTOR);
}

void MapWidget::tempNotificationMarker(qreal x, qreal y)
{
    notificationTimer->stop();
    notificationTimer->singleShot(MM_NOTIFICATION_TIME, this, SLOT(onNotificationTimeExpired()));
    NotificationGraphicsItem* marker = new NotificationGraphicsItem();
    scene->addItem(marker);
    marker->setPos(x, y);
    notificationMarkers.append(marker);
}

void MapWidget::zoomOut()
{
    if(actualZoom >= ZOOM_OUT_LIMIT)
    {
        this->scale(1- ZOOM_OUT_INCREMENT, 1- ZOOM_OUT_INCREMENT);
        actualZoom *= (1-ZOOM_OUT_INCREMENT);
    }
}

void MapWidget::zoomIn(QPoint center)
{
    Q_UNUSED(center);
    if(actualZoom <= ZOOM_IN_LIMIT)
    {
        this->scale(1+ZOOM_IN_INCREMENT, 1+ZOOM_IN_INCREMENT);
        actualZoom *= (1+ZOOM_IN_INCREMENT);
    }
}

void MapWidget::cleanActions(uint robot)
{
    ExploreDirectionGraphicItem *ed = explorationDirections->value(robot, 0);
    if(ed != 0){
        scene->removeItem(ed);
        delete explorationDirections->take(robot);
        ldbg << "BS: Explore direction mission aborted for robot " << robot << endl;
    }
    WaypointMarkerGraphicsItem *w = waypoints->value(robot, 0);
    if(w != 0){
        scene->removeItem(w);
        delete waypoints->take(robot);
        ldbg << "BS: Waypoint removed for robot " << robot << endl;
    }
    ExploreAreaGraphicItem *ea = explorationAreas->value(robot, 0);
    if(ea != 0){
        scene->removeItem(ea);
        delete explorationAreas->take(robot);
        ldbg << "BS: Explore area mission aborted for robot " << robot << endl;
    }
    if(destinations->contains(robot)){
        DestinationGraphicItem* marker = destinations->value(robot);
        marker->hide();
    }
}

void MapWidget::deleteWalls(QList<QGraphicsLineItem *> *walls)
{
    QGraphicsLineItem* wall;
    if(walls != 0){
        while(!walls->isEmpty()){
            wall = walls->takeAt(0);
            scene->removeItem(wall);
            delete wall;
        }
        delete walls;
    }
}

void MapWidget::cleanRobotPaths()
{
    QList<QGraphicsEllipseItem*> *items;
    QGraphicsEllipseItem* pose;
    foreach(uint robot, robotPaths->keys())
    {
        items = robotPaths->value(robot, 0);
        if(items != 0)
        {
            for(int i = 0; i < items->size(); i++)
            {
                pose = items->at(i);
                scene->removeItem(pose);
                delete pose;
            }
            robotPaths->remove(robot);
            delete items;
        }
    }
}

void MapWidget::switchToGlobalView()
{
    this->resize(MAP_WIDGET_WIDTH_GLOBAL, MAP_WIDGET_HEIGHT_GLOBAL);
    this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

void MapWidget::switchToTeleoperationView()
{
    this->resize(MAP_WIDGET_WIDTH_TELEOPERATION, MAP_WIDGET_HEIGHT_TELEOPERATION);
//    this->setMaximumWidth();

}

void MapWidget::setupUi()
{
    QBoxLayout *l = new QBoxLayout(QBoxLayout::LeftToRight,0);
    l->addWidget(this);
//    this->setDragMode(QGraphicsView::ScrollHandDrag);
    this->scale(INITIAL_SCALE_FACTOR , -INITIAL_SCALE_FACTOR);
    this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    //Attenzione: questo è utile per ottimizzare l'indexing, ma non è dinamico! Da rivalutare!
//    scene->setSceneRect(-200,-200,800,800);
    this->show();
}

void MapWidget::drawMap(QList<LineSegment*> lines, uint robotID, bool affidable)
{
    //pulisce quello che c'era già!!!
//    QList<QGraphicsLineItem*> *old = walls->value(robotID, 0);
    deleteWalls(walls->value(robotID, 0));
    walls->remove(robotID);
    foreach(int i, *kenafs){
        if(i != selectedRobot)
        {
            deleteWalls(walls->value(i, 0));
            walls->remove(i);
        }
    }


    QLineF line;
    QList<QGraphicsLineItem*> *temp = new QList<QGraphicsLineItem*>();
    if(robotID == selectedRobot)
    {
        for(int i = 0; i<lines.size(); i++){
            LineSegment *carpaccio = lines[i];
            line = LineSegment(carpaccio->p1() * SLAM_SCALE_FACTOR, carpaccio->p2() * SLAM_SCALE_FACTOR);
            temp->append(scene->addLine(line.x1(), line.y1(), line.x2(), line.y2(), QPen(QColor(Qt::red), 0.6)));
        }
    }
    else
    {
        if(affidable)
        {
            for(int i = 0; i<lines.size(); i++){
                LineSegment *carpaccio = lines[i];
                line = LineSegment(carpaccio->p1() * SLAM_SCALE_FACTOR, carpaccio->p2() * SLAM_SCALE_FACTOR);
                temp->append(scene->addLine(line.x1(), line.y1(), line.x2(), line.y2(), QPen(QColor(Qt::black))));
            }
        }
        else if(!kenafs->contains(robotID)){
            kenafs->append(robotID);
        }
    }
    walls->insert(robotID, temp);
    mappa->append(lines);
}

void MapWidget::drawFrontiers(QList<SLAM::Geometry::Frontier*> frontiers, uint robotID)
{
    //pulisce quello che c'era già!!!
//    QList<QGraphicsLineItem*> *old = this->frontiers->value(robotID, 0);
//    QGraphicsLineItem* front;
//    if(old != 0){
//        while(!old->isEmpty()){
//            front = old->takeAt(0);
//            scene->removeItem(front);
//            delete front;
//        }
//        delete old;
//    }

    if(robotID == selectedRobot)
        {
        QList<QGraphicsLineItem*> *old;
        QGraphicsLineItem* front;
        foreach(uint robot, this->frontiers->keys())
        {
            old = this->frontiers->value(robot, 0);
            if(old != 0){
                while(!old->isEmpty()){
                    front = old->takeAt(0);
                    scene->removeItem(front);
                    delete front;
                }
                delete old;
                this->frontiers->remove(robot);
            }
        }
        QLineF line;
        QList<QGraphicsLineItem*> *temp= new QList<QGraphicsLineItem*>();
        for(int i = 0; i<frontiers.size(); i++){
            LineSegment *carpaccio = frontiers[i];
            line = LineSegment(carpaccio->p1() * SLAM_SCALE_FACTOR, carpaccio->p2() * SLAM_SCALE_FACTOR);
            temp->append(scene->addLine(line.x1(), line.y1(), line.x2(), line.y2(), QPen(Qt::DashLine)));
        }
        this->frontiers->insert(robotID, temp);
    }
}

void MapWidget::drawPath(QList<SLAM::PathNode *> poses, uint robot)
{
    if(robot == selectedRobot)
    {
        cleanRobotPaths();
        SLAM::PathNode * pose;
        QList<QGraphicsEllipseItem*> *items = new QList<QGraphicsEllipseItem*>();
        for(int i = 0; i < poses.size(); i++)
        {
            pose = poses.at(i);
            items->append(scene->addEllipse(pose->getX() * SLAM_SCALE_FACTOR,
                                            pose->getY() * SLAM_SCALE_FACTOR, 0.5, 0.5, QPen(QColor(Qt::green))));
        }
        robotPaths->insert(robot, items);
    }
}

void MapWidget::addSemanticInfoToMap(Data::Pose coordinates, QString info)
{
}

void MapWidget::showRobot(Data::Pose place, bool selected, uint robotID) //TODO
{
    RobotGraphicItem * robot;
    robot = robots->value(robotID, 0);
    if(robot == 0)
    {
         robot = new RobotGraphicItem(robotID,selected);
         robots->insert(robotID,robot);
         connect(robot, SIGNAL(sigSelectedRobotChangedRGI(uint)), this, SLOT(onRobotSelected(uint)));
         scene->addItem(robot);
    }
    robot->setPos(place.getX()*SLAM_SCALE_FACTOR /*- ROBOT_MARKER_WIDTH/2*/,
                   place.getY()*SLAM_SCALE_FACTOR /*- ROBOT_MARKER_HEIGHT/2*/);
    robot->setRotation(place.getTheta()*180/(M_PI));
    ExploreDirectionGraphicItem *e = explorationDirections->value(robotID, 0);
    if(e != 0){
        e->setPos(place.getX()*SLAM_SCALE_FACTOR, place.getY()*SLAM_SCALE_FACTOR );
    }
}

void MapWidget::showVictim(Data::Pose place, uint victimID)
{
    VictimGraphicItem* victim = new VictimGraphicItem(0,victimID, this);
    victims->insert(victimID, victim);
    connect(victim, SIGNAL(signalSelectedVictim(uint)), this, SLOT(onVictimSelected(uint)));
    connect(victim, SIGNAL(signalVictimMoved(uint,Data::Pose)), this, SLOT(onVictimMoved(uint,Data::Pose)));
    victim->setPos(place.getX()*SLAM_SCALE_FACTOR - VICTIM_MARKER_WIDTH/2,
                   place.getY()*SLAM_SCALE_FACTOR - VICTIM_MARKER_HEIGHT/2);
    scene->addItem(victim);
}

void MapWidget::cleanMap()
{
    QGraphicsLineItem* wall;
    foreach(uint robot, walls->keys())
    {
        QList<QGraphicsLineItem*> *xWalls = walls->take(robot);
        while(!xWalls->isEmpty()){
            wall = xWalls->takeAt(0);
            scene->removeItem(wall);
            delete wall;
        }
        delete xWalls;
    }
    QGraphicsLineItem* frontier;
    foreach(uint robot, frontiers->keys())
    {
        QList<QGraphicsLineItem*> *xFrontiers = frontiers->take(robot);
        while(!xFrontiers->isEmpty()){
            frontier = xFrontiers->takeAt(0);
            scene->removeItem(frontier);
            delete frontier;
        }
        delete xFrontiers;
    }
    foreach(uint key, robots->keys())
        scene->removeItem(robots->value(key));
    foreach(uint k, waypoints->keys())
        scene->removeItem(waypoints->value(k));
    mappa->clear();
}

void MapWidget::cleanRobots()
{
    //TODO... is it useful?
}

void MapWidget::deleteVictim(uint id)
{
    ldbg << "I'm deleting victim " << id << "from map" << endl;
    VictimGraphicItem* victim = victims->value(id, 0);
    if(victim != 0)
    {
        scene->removeItem(victim);
        victims->remove(id);
        delete victim; //SURE?
    }
}

//------------------------------------------------------Event Handlers----------------------------//

void MapWidget::wheelEvent(QWheelEvent *event)
{ //To be refined with some granularity of steps!!!
    wheel = event;
    int d = wheel->delta();
    QPoint center = wheel->pos();
    if(d > 0)
        zoomIn(center);
    if(d < 0)
        zoomOut();
    wheel->accept();
}

void MapWidget::saveSVG()
{
     QString newPath = QFileDialog::getSaveFileName(this, tr("Save SVG"),
                                                    "/" , tr("SVG files (*.svg)"));
     if (newPath.isEmpty())
         return;
     QSvgGenerator generator;
     generator.setFileName(newPath);
     generator.setSize(QSize(200, 200));
     generator.setViewBox(QRect(0, 0, 200, 200));
     generator.setTitle(tr("SVG Map for Mr Mazu"));
     generator.setDescription(tr("Marameo maramà!"));
     QPainter painter;
     painter.begin(&generator);
     QPen p = QPen();
     p.setWidth(1);
     painter.setPen(p);
     scene->render(&painter);
     painter.end();

}

void MapWidget::saveForMods()
{
    QString newPath = QFileDialog::getSaveFileName(this, tr("Save Map"),
                                                   "/" , tr("TXT files (*.txt)"));
    if (newPath.isEmpty())
        return;
    QFile file(newPath);
    if (file.open(QFile::WriteOnly)) {
         QTextStream out(&file);
         out << "Walls:" << endl;
         for(int i = 0; i< mappa->size(); i++){
             out << "(" << mappa->at(i)->x1() << "," << mappa->at(i)->y1() << ")" <<
                    "(" << mappa->at(i)->x2() << "," << mappa->at(i)->y2() << ")" << endl; //TODO
         }
         out << "Victims:" << endl;
         foreach(VictimGraphicItem * victim, victims->values()){
             out << "(" << (victim->pos().x()/SLAM_SCALE_FACTOR) << "," <<
                    (victim->pos().y()/SLAM_SCALE_FACTOR) <<")" << endl;
         }
         //Aggiungo le vittime! E le stanze quando ci saranno!
    }
}

void MapWidget::onRobotSelected(uint id)
{
    if(id != selectedRobot){
        foreach(RobotGraphicItem* robot, robots->values())
        {
            if(robot->getRobotID() != id)
            {
                robot->deselect();
                if(destinations->contains(robot->getRobotID())){
                    DestinationGraphicItem *marker = destinations->value(robot->getRobotID());
                    marker->deselect();
                }
            }
        }
        WaypointMarkerGraphicsItem *oldWaypoint = waypoints->value(id, 0);
        if(oldWaypoint != 0)
            oldWaypoint->select();
        WaypointMarkerGraphicsItem *newWaypoint  = waypoints->value(selectedRobot, 0);
        if(newWaypoint != 0)
            newWaypoint->deselect();
        this->selectedRobot = id;
        if(destinations->contains(id)){
            DestinationGraphicItem *marker = destinations->value(id);
            marker->deselect();
        }
        cleanRobotPaths();
        emit signalRobotSelected(id);
    }
}

void MapWidget::onVictimSelected(uint id)
{
    if(id != selectedVictim){
        VictimGraphicItem* v = victims->value(selectedVictim, 0);
        if(v != 0)
        {
            v->deselect();
            scene->update(QRect(v->pos().x(), v->pos().y(), 1, 1));
        }
        v = victims->value(id, 0);
        if(v != 0){
            v->select();
            scene->update(QRect(v->pos().x(), v->pos().y(), 1, 1));
        }
        this->selectedVictim = id;
        emit signalVictimSelected(id);
    }
}

void MapWidget::onSelectedRobotChanged(uint id)
{
    if(id != selectedRobot){
        foreach(RobotGraphicItem* robot, robots->values())
        {
            if(robot->getRobotID() != id)
            {
                robot->deselect();
                if(destinations->contains(robot->getRobotID())){
                    DestinationGraphicItem *marker = destinations->value(robot->getRobotID());
                    marker->deselect();
                }
            }
            else
            {
                robot->select();
                centerOn(robot);
                if(destinations->contains(robot->getRobotID())){
                    DestinationGraphicItem *marker = destinations->value(robot->getRobotID());
                    marker->select();
                }
            }
        }
        WaypointMarkerGraphicsItem *oldWaypoint = waypoints->value(id, 0);
        if(oldWaypoint != 0){
            oldWaypoint->select();
        }
        WaypointMarkerGraphicsItem *newWaypoint  = waypoints->value(selectedRobot, 0);
        if(newWaypoint != 0){
            newWaypoint->deselect();
        }
        this->selectedRobot = id;
        cleanRobotPaths();
    }
}

void MapWidget::setRobotUnreachable(uint id)
{
    RobotGraphicItem * robot = robots->value(id, 0);
    if(robot != 0)
    {
        robot->setReachable(false);
        if(destinations->contains(id)){
            DestinationGraphicItem* marker = destinations->value(id);
            marker->hide();
        }
    }
}

void MapWidget::setRobotReachable(uint id)
{
    RobotGraphicItem * robot = robots->value(id, 0);
    if(robot != 0)
    {
        robot->setReachable(true);
        if(destinations->contains(id)){
            DestinationGraphicItem* marker = destinations->value(id);
            marker->show();
        }
    }
}

void MapWidget::setRobotDestination(uint robotID, Data::Pose position)
{
    DestinationGraphicItem* marker;
    if(destinations->contains(robotID)){
        marker = destinations->value(robotID);
        marker->show();
    }
    else{
        marker = new DestinationGraphicItem(robotID);
        destinations->insert(robotID, marker);
        scene->addItem(marker);
    }
    if(robotID == selectedRobot)
        marker->select();
    marker->setPos(position.getX() * SLAM_SCALE_FACTOR, position.getY() * SLAM_SCALE_FACTOR);
}

void MapWidget::onWaypointSetted(Data::Pose pose)
{
    cleanActions(selectedRobot);
    WaypointMarkerGraphicsItem *waypoint = waypoints->value(selectedRobot, 0);
    if(waypoint == 0){
        waypoint = new WaypointMarkerGraphicsItem();
        waypoints->insert(selectedRobot, waypoint);
        scene->addItem(waypoint);
    }
    waypoint->select();
    waypoint->setPos(pose.getX() * SLAM_SCALE_FACTOR, pose.getY() * SLAM_SCALE_FACTOR);
    emit signalAddWaypoint(pose, this->selectedRobot);
}

void MapWidget::onVictimMoved(uint id,const Data::Pose &position)
{
    emit signalVictimMoved(id, position);
}

void MapWidget::onCustomLabelAdded(Data::Pose pos, QString mex)
{
    CustomLabelGraphicItem *l = new CustomLabelGraphicItem(this, mex);
    l->setPos(pos.getX() * SLAM_SCALE_FACTOR, pos.getY() * SLAM_SCALE_FACTOR);
    scene->addItem(l);
    customLabels->append(l);
}

void MapWidget::onDangerLabelAdded(Data::Pose pos)
{
    DangerGraphicsItem *l = new DangerGraphicsItem();
    l->setPos(pos.getX() * SLAM_SCALE_FACTOR, pos.getY() * SLAM_SCALE_FACTOR);
    scene->addItem(l);
    dangerLabels->append(l);
}

void MapWidget::onVictimAddedByOperator(Data::Pose position)
{
    emit signalVictimAdded(position);
}

void MapWidget::onMessageInteraction(int messageId, QString module, QString result)
{
    for(int i = 0; i < popupMessages->size(); i++){
        PopupWidget *p = popupMessages->at(i);
        if(p->getId() == messageId){
            if(!popupMessages->removeOne(p))
                ldbg << "some error occurred while removing the popup message" << endl;
            p->close();
            p->deleteLater();
        }
    }
    emit signalPopupMessageArchived(messageId, module, result);
}

void MapWidget::onExploreDirection(double x, double y)
{
    //gestisco il marker della direzione di esplorazione...
    /*
        - Bisogna gestire anche l'informazione della missione a livello di singlerobotInfo
    */
    cleanActions(selectedRobot);
    RobotGraphicItem *robot = robots->value(selectedRobot,0);
    if(robot == 0)
        return;
    //draw the marker:
        ExploreDirectionGraphicItem *marker = explorationDirections->value(selectedRobot, 0);
        if(marker == 0){
            marker = new ExploreDirectionGraphicItem(x*10, y*10);
            explorationDirections->insert(selectedRobot, marker);
            scene->addItem(marker);
        }
        else{
            marker->updateVector(x*5, y*5);
        }
        QPointF pose = robot->pos();
        marker->setPos(pose.x(), pose.y());
        explorationDirections->insert(selectedRobot, marker);
    emit signalDirectionHighLevelCommand(x, y, selectedRobot);
}

void MapWidget::onExploreArea(double x, double y)
{
    /*
        - Bisogna gestire anche l'informazione della missione a livello di singlerobotInfo
    */
    cleanActions(selectedRobot);
    ExploreAreaGraphicItem *marker = new ExploreAreaGraphicItem();
    explorationAreas->insert(selectedRobot, marker);
    scene->addItem(marker);
    marker->setPos(x,y);
    emit signalAreaHighLevelCommand(x * SLAM_SCALE_FACTOR, y * SLAM_SCALE_FACTOR, selectedRobot);
}

void MapWidget::onWaypointReached(uint robot)
{
    cleanActions(robot);
//    scene->removeItem(waypoints->value(robot));
//    delete waypoints->take(robot);
    ldbg << "waypoint reached" << endl;
}

void MapWidget::dragEnterEvent(QGraphicsSceneDragDropEvent *event)
{
    if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
             if (event->source() == this) {
                 event->setDropAction(Qt::MoveAction);
                 event->accept();
             } else {
                 event->acceptProposedAction();
             }
    } else {
        // Gestione dei comandi di alto livello!
           event->ignore();
    }
}

void MapWidget::dragLeaveEvent(QGraphicsSceneDragDropEvent *event)
{
    Q_UNUSED(event);
    update();
}

void MapWidget::dropEvent(QDropEvent *event)
{
    if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
             if (event->source() == this) {
                 event->setDropAction(Qt::MoveAction);
                 event->accept();
                 QByteArray itemData = event->mimeData()->data("application/x-dnditemdata");
                 QDataStream dataStream(&itemData, QIODevice::ReadOnly);
                 uint victimID;
                 dataStream >> victimID;
                 VictimGraphicItem* victim = victims->value(victimID, 0);
                 if(victim == 0){
                     ldbg << "error!" << endl;
                     return;
                 }
                 int ret = QMessageBox::warning(0, tr("Victim repositioning"),
                                                 "Are you sure you want to re-position victim " +
                                                    QString::number(victim->getVictimID()) + "?",
                                                 QMessageBox::Yes | QMessageBox::No);
                 if(ret == QMessageBox::Yes){
                     victim->setPos(mapToScene(event->pos()));
                     emit signalVictimMoved(victim->getVictimID(), Data::Pose(mapToScene(event->pos()).x()/SLAM_SCALE_FACTOR,
                                                                 mapToScene(event->pos()).y()/SLAM_SCALE_FACTOR, 0));
                 }
                 else
                     victim->setPos(victim->pos());
                 update();
             } else {
                 event->acceptProposedAction();
             }
         } else {
             event->ignore();
         }
}

void MapWidget::dragMoveEvent(QDragMoveEvent *event)
{
    if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
             if (event->source() == this) {
                 event->setDropAction(Qt::MoveAction);
                 event->accept();
             } else {
                 event->acceptProposedAction();
             }
         } else {
             event->ignore();
         }
}

void MapWidget::showPopup(int id, Data::Pose position, QString module, QString message)
{
    PopupWidget* mex = new PopupWidget(0, id, module, message);
    QGraphicsProxyWidget *proxy = scene->addWidget(mex, Qt::Widget);
    popupMessages->append(mex);
    proxy->scale(0.15, -0.15);
    proxy->setPos(proxy->size().width() + position.getX(),
                  proxy->size().height() + position.getY());
    connect(mex, SIGNAL(signalOperatorBehaviour(int,QString,QString)),
            this, SLOT(onMessageInteraction(int,QString,QString)));
}

void MapWidget::onMessageRemoval(int id)
{
    for(int i = 0; i < popupMessages->size(); i++){
        PopupWidget *p = popupMessages->at(i);
        if(p->getId() == id){
            if(!popupMessages->removeOne(p))
                ldbg << "some error occurred while removing the popup message" << endl;
            p->close();
            delete p;
        }
    }
}

void MapWidget::onNotificationTimeExpired()
{
    NotificationGraphicsItem* marker;
    while(!notificationMarkers.isEmpty())
    {
        marker = notificationMarkers.takeFirst();
        scene->removeItem(marker);
        delete marker;
    }
}

}

