#include "basestationcoreui.h"
#include <QDockWidget>
#include <QHBoxLayout>
#include <QAction>
#include <QMenuBar>
#include <QTabWidget>
#include <QKeyEvent>


namespace graphics{

BaseStationCoreUi::BaseStationCoreUi(QWidget *parent, TeleoperationWidget *t, AirTeleoperationWidget *a,
                                     QLabel *focusCamera, QTabWidget *messages, SingleRobotInfoWidget *robotInfo, uint nRobots) :
    QMainWindow(parent), thumbnailsLayout(new QHBoxLayout()),
    teleoperationWidget(t), airTeleoperationWidget(a),
    focusCamera(focusCamera), thumbnails(new QWidget(this)),
    messageViewer(messages), teleoperationDock(new QDockWidget(tr("Teleoperation"), this)),
    thumbnailsDock(new QDockWidget(tr("Camera thumbnails"), this)),
    rightDock(new QDockWidget(tr("Single robot Info"), this)),
    messageDock(new QDockWidget(tr("message manager"), this)),
    bigCameraDock(new QDockWidget(tr("Camera View"), this)),
    singleRobotInfo(robotInfo),
    currentRobot(0), aerialTeleoperation(false), nRobots(nRobots)
{
    this->setupUi();
    connect(teleoperationWidget, SIGNAL(signalSwitchRobotType()), this, SLOT(onSwitchRobotType()));
    connect(airTeleoperationWidget, SIGNAL(signalSwitchRobotType()), this, SLOT(onSwitchRobotType()));
    this->grabKeyboard();
}

BaseStationCoreUi::~BaseStationCoreUi()
{
}

void BaseStationCoreUi::setupUi()
{
    //FILE menu creation
    saveSVGEAction = new QAction(tr("Save Map to SVG"), this);
    QMenu *fileMenu = this->menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(saveSVGEAction);
    saveMapForTestsAction = new QAction(tr("Save Map for further tests"), this);
    fileMenu->addAction(saveMapForTestsAction);

    showConfigurationInterfaceAction = new QAction(tr("Configure..."), this);
    QMenu *configureMenu = this->menuBar()->addMenu(tr("&Preferences"));
    configureMenu->addAction(showConfigurationInterfaceAction);
    connect(showConfigurationInterfaceAction, SIGNAL(triggered()),
            this, SLOT(onShowConfigurationGUIDemand()));

    //Clean map features
    QMenu *advancedMenu = this->menuBar()->addMenu(tr("&Advanced"));
    QAction *cleanMap = new QAction(tr("Clean Map of the selected robot"), this);
    advancedMenu->addAction(cleanMap);
    connect(cleanMap, SIGNAL(triggered()), this, SLOT(onMapTrouble()));

    teleoperationDock->setWidget(teleoperationWidget);
    addDockWidget(Qt::TopDockWidgetArea, thumbnailsDock);
    addDockWidget(Qt::TopDockWidgetArea, teleoperationDock);
    addDockWidget(Qt::RightDockWidgetArea, bigCameraDock);
    addDockWidget(Qt::RightDockWidgetArea, rightDock);
    addDockWidget(Qt::BottomDockWidgetArea, messageDock);
    this->setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
    this->setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    QWidget* singleRobotWidget = new QWidget();
    QVBoxLayout *l = new QVBoxLayout();
//    l->addWidget(focusCamera);
    l->addWidget(singleRobotInfo);
//    focusCamera->setParent(singleRobotWidget);
    singleRobotInfo->setParent(singleRobotWidget);
    singleRobotWidget->setLayout(l);

    thumbnails->setLayout(thumbnailsLayout);
    thumbnailsDock->setWidget(thumbnails);
    rightDock->setWidget(singleRobotWidget);
    bigCameraDock->setWidget(focusCamera);
    bigCameraDock->setLayout(new QGridLayout());
    bigCameraDock->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    messageDock->setWidget(messageViewer);

    connect(bigCameraDock, SIGNAL(dockLocationChanged(Qt::DockWidgetArea)), this, SLOT(onDockPositionChanged()));

    this->hide();
}

void BaseStationCoreUi::keyPressEvent(QKeyEvent *e)
{
    //qDebug("Received!");
    bool handled = false;
    if(!e->isAutoRepeat() && ((e->key() == Qt::Key_N) || (e->key() == Qt::Key_B)))
    {
        emit sigSelectedRobotChangedBCUM(
                    (currentRobot + ((e->key() == Qt::Key_N) - (e->key() == Qt::Key_B))) % nRobots);
        e->accept();
        handled = true;
    }
    int keyZero = Qt::Key_0;
    int n = nRobots;
    if(!e->isAutoRepeat() && ((e->key() >= Qt::Key_0) && e->key() < (keyZero + n))){
        emit sigSelectedRobotChangedBCUM(e->key() - Qt::Key_0);
        e->accept();
        handled = true;
    }
//    e->ignore();
    //Trasmetto l'evento ai singoli pazzi widget!
    if(!handled)
    {
        teleoperationWidget->handleKeyPressEvent(e);
        airTeleoperationWidget->handleKeyPressEvent(e);
        mapWidget->handleKeyPressEvent(e);
    }
    //qDebug("Managed!");
}

void BaseStationCoreUi::keyReleaseEvent(QKeyEvent *e)
{
    teleoperationWidget->handleKeyReleaseEvent(e);
    airTeleoperationWidget->handleKeyReleaseEvent(e);
    mapWidget->handleKeyReleaseEvent(e);
}

void BaseStationCoreUi::onSwitchToGlobal()
{
    this->showMaximized();
}

void BaseStationCoreUi::onSwitchToTeleoperation()
{
}

void BaseStationCoreUi::onSelectedRobotChanged(uint newRobotID)
{
    this->currentRobot = newRobotID;
}

void BaseStationCoreUi::addCameraThumbnail(QLabel *cameraView)
{
    this->thumbnailsLayout->addWidget(cameraView);
}

void BaseStationCoreUi::setCentralWidget(QWidget *widget)
{
    this->mapWidget = (MapWidget*)widget;
    QMainWindow::setCentralWidget(widget);
    connect(saveSVGEAction, SIGNAL(triggered()), mapWidget, SLOT(saveSVG()));
    connect(saveMapForTestsAction, SIGNAL(triggered()), mapWidget, SLOT(saveForMods()));
}

void BaseStationCoreUi::onShowConfigurationGUIDemand()
{
    emit siglShowConfigurationGUI();
}

void BaseStationCoreUi::onSwitchRobotType()
{
    if(aerialTeleoperation)
    {
        aerialTeleoperation = false;
        teleoperationDock->setWidget(teleoperationWidget);
    }
    else
    {
        aerialTeleoperation = true;
        teleoperationDock->setWidget(airTeleoperationWidget);
    }
    airTeleoperationWidget->setActive(aerialTeleoperation);
    teleoperationWidget->setActive(!aerialTeleoperation);
}

void BaseStationCoreUi::onDockPositionChanged()
{
    bigCameraDock->resize(FOCUS_CAMERA_WIDTH, FOCUS_CAMERA_HEIGHT);
    bigCameraDock->widget()->resize(FOCUS_CAMERA_WIDTH, FOCUS_CAMERA_HEIGHT);
}

void BaseStationCoreUi::onMapTrouble()
{
    emit signalMapNotWorking(this->currentRobot);
}

}
