#include "mapgraphicsscene.h"
#include <QGraphicsSceneMouseEvent>
#include <QInputDialog>
#include <QPainter>
#include <QGraphicsSceneDragDropEvent>
#include <QDrag>
#include <QCursor>
#include <QApplication>
#include <QMimeData>
#include "baseStation/graphicparams.h"

namespace graphics{

MapGraphicsScene::MapGraphicsScene(QWidget *widget):
    mapWidget(widget), parentWidget(widget), dragging(false), highLevelCommand(false),
    currentHighLevelCommandItem(NULL),
    actionTimer(new QTimer())
{
    menu = new QMenu(tr("Add Feature"));
    addVictimAction = new QAction(tr("Add a victim here"), this);
    addLabelAction = new QAction(tr("Add custom label here"), this);
    addDangerLabel = new QAction(tr("Add danger label here"), this);
    menu->addAction(addVictimAction);
    menu->addAction(addLabelAction);
    menu->addAction(addDangerLabel);
    //Connetions for the actions!!!
    connect(addVictimAction, SIGNAL(triggered()), this, SLOT(onAddVictim()));
    connect(addLabelAction, SIGNAL(triggered()), this, SLOT(onAddLabel()));
    connect(addDangerLabel, SIGNAL(triggered()), this, SLOT(onAddDanger()));
}

void MapGraphicsScene::setParentWidget(QWidget *parentWidget)
{
    this->parentWidget = parentWidget;
}

void MapGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    dragging = false;
    if(event->buttons() == Qt::RightButton){
        highLevelCommand = false;
    }
    else
        QGraphicsScene::mousePressEvent(event); //call to the original event handler
}


void MapGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(event->buttons() == Qt::RightButton)
    {
        if (QLineF(event->screenPos(),
                   event->buttonDownScreenPos(Qt::LeftButton)).length() < QApplication::startDragDistance())
        {
            return;
        }
        else{
            dragging = true;
            if(currentHighLevelCommandItem == NULL)
            {
                currentHighLevelCommandItem = new ExploreDirectionGraphicItem(
                            event->scenePos().x() - event->buttonDownScenePos(Qt::RightButton).x(),
                            event->scenePos().y() - event->buttonDownScenePos(Qt::RightButton).y());
                this->addItem(currentHighLevelCommandItem);
                currentHighLevelCommandItem->setPos(event->scenePos());
            }
            else
            {
                currentHighLevelCommandItem->updateVector(
                            event->scenePos().x() - event->buttonDownScenePos(Qt::RightButton).x(),
                            event->scenePos().y() - event->buttonDownScenePos(Qt::RightButton).y());
            }
        }
    }
    else
        QGraphicsScene::mouseMoveEvent(event);
}

void MapGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    bool handled = false;
    if(dragging){
        handled = true;
        //explore direction case:
        //EMIT the proper high level command signal!!!
        double x = currentHighLevelCommandItem->getX();
        double y = currentHighLevelCommandItem->getY();
        emit signalExploreDirection(x/(abs(x)+abs(y)), y/(abs(x)+abs(y)));
        //then delete the temporal action marker!
        if(currentHighLevelCommandItem != NULL)
        {
            this->removeItem(currentHighLevelCommandItem);
            delete currentHighLevelCommandItem;
            currentHighLevelCommandItem = NULL;
        }
        dragging = false;
    }
    else{
        //handling of the right click event
        if(event->button() == Qt::RightButton)
        {
            handled = true;
            waypointPose = Data::Pose(event->buttonDownScenePos(Qt::RightButton).x() / SLAM_SCALE_FACTOR,
                                      event->buttonDownScenePos(Qt::RightButton).y() / SLAM_SCALE_FACTOR, 0);
            if(highLevelCommand == false)
                actionTimer->singleShot(800,this, SLOT(onWaypointCommand()));
//            emit signalWaypoint(waypointPose);
        }
        else if(event->button() == Qt::MiddleButton){
            handled = true;
            menu->popup(event->screenPos());
            eventScenePos = Data::Pose(event->buttonDownScenePos(Qt::MiddleButton).x() / SLAM_SCALE_FACTOR,
                                       event->buttonDownScenePos(Qt::MiddleButton).y() / SLAM_SCALE_FACTOR, 0);
        }
    }
    if(!handled)
        QGraphicsScene::mouseReleaseEvent(event);
}

void MapGraphicsScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    //EMIT areaHighLevelCommand
    if(event->button() == Qt::RightButton)
    {
        emit signalExploreArea(event->buttonDownScenePos(Qt::RightButton).x(),
                               event->buttonDownScenePos(Qt::RightButton).y());
        highLevelCommand = true;
    }
}

void MapGraphicsScene::onAddLabel()
{
//    bool ok;
//    QString text = QInputDialog::getText(mapWidget, tr("Insert Custom Label"),
//                                              tr("Label:"), QLineEdit::Normal,
//                                              "", &ok);
//    if (ok && !text.isEmpty())
    emit signalLabelAdded(eventScenePos, "");
}

void MapGraphicsScene::onAddDanger()
{
    emit signalDangerLabelAdded(eventScenePos);
}

void MapGraphicsScene::onWaypointCommand()
{
    if(highLevelCommand == false)
        emit signalWaypoint(waypointPose);
}

void MapGraphicsScene::onAddVictim()
{
    emit signalVictimAddedByUser(eventScenePos);
}

}
