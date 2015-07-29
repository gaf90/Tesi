#ifndef MAPGRAPHICSSCENE_H
#define MAPGRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QMenu>
#include <QTimer>
#include "data/pose.h"
#include "graphics/utils/exploredirectiongraphicitem.h"


namespace graphics{

class MapGraphicsScene: public QGraphicsScene
{
    Q_OBJECT
public:
    MapGraphicsScene(QWidget *widget = 0);

    void setParentWidget(QWidget* parentWidget);

signals:

    void signalWaypoint(Data::Pose pose);
    void signalVictimAddedByUser(Data::Pose position);
    void signalLabelAdded(Data::Pose position, QString mex);
    void signalDangerLabelAdded(Data::Pose position);
    //High level commands
    void signalExploreDirection(double x, double y);
    void signalExploreArea(double x, double y);

private slots:
    void onAddVictim();
    void onAddLabel();
    void onAddDanger();
    void onWaypointCommand();

private:

    //! overrided function. Handles mouse clicks
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

    QMenu *menu;
    QAction *addVictimAction;
    QAction *addLabelAction;
    QAction *addDangerLabel;
    Data::Pose eventScenePos;
    QWidget *mapWidget;
    QWidget *parentWidget;
    bool dragging;
    bool highLevelCommand;
    ExploreDirectionGraphicItem *currentHighLevelCommandItem;
    QTimer *actionTimer;
    Data::Pose waypointPose;
};

}
#endif // MAPGRAPHICSSCENE_H
