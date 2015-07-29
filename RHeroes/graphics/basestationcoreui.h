#ifndef BASESTATIONCOREUI_H
#define BASESTATIONCOREUI_H

#include <QMainWindow>
#include <QLabel>
#include <QHBoxLayout>
#include "graphics/modules/mapwidget.h"
#include "graphics/modules/teleoperationwidget.h"
#include "graphics/modules/airteleoperationwidget.h"
#include "graphics/modules/singlerobotinfowidget.h"

namespace graphics{

class BaseStationCoreUi : public QMainWindow
{
    Q_OBJECT
public:
    explicit BaseStationCoreUi(QWidget *parent = 0, TeleoperationWidget *t = 0,
                               AirTeleoperationWidget *a = 0, QLabel *focusCamera = 0,
                               QTabWidget *messages = 0, SingleRobotInfoWidget *robotInfo = 0, uint nRobots = 0);

    virtual ~BaseStationCoreUi();

    void addCameraThumbnail(QLabel* cameraView);

    void setCentralWidget(QWidget *widget);

signals:

    /**
    * This signal is emitted when the operator request to switch from the globa UI
    * to the teleoperation UI.
    */
    void signalSwitchToTeleoperation();

    /**
    * This signal is emitted when the operator request to switch from the teleoperation UI
    * to the global UI.
    */
    void signalSwitchToGlobal();

    /**
    * This signal is emitted when the operator selects a (different) robot.
    * @param the ID of the selected robot.
    */
    void sigSelectedRobotChangedBCUM(uint robotID);

    /**
    * This signal is emitted when the user wants to modify some configuration param or
    * activate/deactivate some module.
    */
    void siglShowConfigurationGUI();

    void signalMapNotWorking(uint robotID);

public slots:

    /**
    * This method is used to manage the switching process to the Teleoperation UI.
    */
    void onSwitchToTeleoperation();

    /**
    * This method is used to manage the switching process to the Global UI.
    */
    void onSwitchToGlobal();

    /**
    * This slot handles the selection of a diffferent robot by the operator.
    * @param the id of the new selected robot.
    */
    void onSelectedRobotChanged(uint newRobotID);

private slots:
    void onShowConfigurationGUIDemand();

    void onSwitchRobotType();

    void onDockPositionChanged();

    void onMapTrouble();

private:
    void setupUi();

    void keyPressEvent(QKeyEvent *);

    void keyReleaseEvent(QKeyEvent * e);

    //components & Widgets
    QHBoxLayout *thumbnailsLayout;
    TeleoperationWidget *teleoperationWidget;
    AirTeleoperationWidget *airTeleoperationWidget;
    QLabel *focusCamera;
    QWidget *thumbnails;
    MapWidget *mapWidget;
    QTabWidget *messageViewer;

    QAction *saveSVGEAction;
    QAction *saveMapForTestsAction;
    QAction *showConfigurationInterfaceAction;

    //QDockWidgets
    QDockWidget *teleoperationDock;
    QDockWidget *thumbnailsDock;
    QDockWidget *rightDock;
    QDockWidget *messageDock;
    QDockWidget *bigCameraDock;

    SingleRobotInfoWidget *singleRobotInfo;

    uint currentRobot;

    bool aerialTeleoperation;

    uint nRobots;

};


}
#endif // BASESTATIONCOREUI_H
