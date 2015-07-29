#include "basestationcore.h"

using namespace graphics;

namespace BaseStation{

BaseStationCore::BaseStationCore(QObject *parent) :
    QObject(parent),
    mapManager(0),
    cameraManager(new CameraManager()),
    notificationManager(new NotificationManager()),
    victimManager(new VictimManager()),
    configurationManager(new ConfigurationManager(this)),
    baseStationWSS(0),
    baseStationCoreUI(0),
    spawnUi(new SpawnUI()),
    selectedRobotID(0),
    semanticMappingManager(0),
    teleoperationWidget(new TeleoperationWidget()),
    airTeleoperationWidget(new AirTeleoperationWidget()),
    kenafs(new QList<uint>())
{
    connect(spawnUi, SIGNAL(sigApplicationStarted(uint)), this, SLOT(onStartedApplication(uint)));
    connect(spawnUi, SIGNAL(sigApplicationConfigure(uint)), this, SLOT(onConfigureApplication(uint)));

    spawnUi->show();
}

BaseStationCore::~BaseStationCore()
{
    delete this->baseStationWSS;
    delete this->mapManager;
    delete this->spawnUi;
}

void BaseStationCore::onSwitchToTeleoperation()
{
}

void BaseStationCore::onSwitchToGlobal()
{
}

/*-----------------------------------------------------------------------------------------------
                    This method is used after the deployment phase!
-----------------------------------------------------------------------------------------------*/
void BaseStationCore::onStartedApplication(uint nActiveRobots)
{

    /*Init Modules*/
    singleRobotInfoManager = new InfoManager(this, nActiveRobots);
    mapManager = new MapManager(0, kenafs);
    QTabWidget *messageManagerViewer = notificationManager->getMessageManagerViewer();
    messageManagerViewer->addTab(victimManager->getVictimsView(),"Victims");
    baseStationCoreUI = new BaseStationCoreUi(0, teleoperationWidget, airTeleoperationWidget, cameraManager->getFocusView(), messageManagerViewer,
                                   singleRobotInfoManager->getSingleRobotInfoWidget(), nActiveRobots);

    baseStationCoreUI->setCentralWidget(mapManager->getUi());
    for(uint i=0; i<nActiveRobots; i++)
    {
        baseStationCoreUI->addCameraThumbnail(cameraManager->getThumbnail(i));
    }

    baseStationWSS->onFramerateSetting(0, HIGH_FRAMERATE);

    /*MESSAGES*/

    //Teleoperation Widget --> Basestation WSS (Send new teleoperation command)
    connect(teleoperationWidget,SIGNAL(sigSendTeleoperationCommand(double,double,uint)),
            baseStationWSS, SLOT(onSendTeleoperationCommand(double,double,uint)));

    //Air Teleoperation Widget --> Basestation WSS (Send new air teleoperation command)
    connect(airTeleoperationWidget, SIGNAL(sigSendAirTeleoperationCommand(double,double,double,double,uint)),
            baseStationWSS, SLOT(onSendAirTeleoperationCommand(double,double,double,double,uint)));

    //Info Manager --> Basestation WSS (Send new information request command)
    connect(singleRobotInfoManager, SIGNAL(sigSendInfoMessage(uint)),
            baseStationWSS, SLOT(onSendInfoMessage(uint)));

    //Map Manager --> Basestation WSS (Send waypoint command)
    connect(mapManager, SIGNAL(sigWaypointCommand(Data::Pose,bool,uint,uint)),
            baseStationWSS, SLOT(onWaypointCommand(Data::Pose,bool,uint,uint)));

    //Map Manager --> Basestation WSS (Send high level command)
    connect(mapManager, SIGNAL(sigHighLevelCommandMM(double,double,bool,uint,uint,bool)),
            baseStationWSS, SLOT(onHighLevelCommand(double,double,bool,uint,uint,bool)));

    //Basestation Core UI --> Basestation WSS (Map not working)
    connect(baseStationCoreUI, SIGNAL(signalMapNotWorking(uint)),
            baseStationWSS, SLOT(onMapNotWorking(uint)));

    //Basestation WSS --> Camera Manager (Received new images from Upis)
    connect(baseStationWSS, SIGNAL(sigCameraDataReceived(uint,QImage)),
            cameraManager,SLOT(onCameraDataReceived(uint,QImage)));

    //Basestation WSS --> Info Manager (Received new info message)
    connect(baseStationWSS,SIGNAL(sigInfoMessageReceived(Data::InfoMessage,uint)),
            singleRobotInfoManager,SLOT(onNewInfoMessageReceived(Data::InfoMessage,uint)));

    //Basestation WSS --> Map Manager (Received new map data)
    connect(baseStationWSS, SIGNAL(sigNewMapDataReceived(SLAM::Map,uint)),
            mapManager, SLOT(onNewMapDataReceived(SLAM::Map, uint)));

    //Basestation WSS --> Map Manager (Received new destination data)
    connect(baseStationWSS, SIGNAL(sigNewDestinationReceived(uint,Data::Pose)),
            mapManager, SLOT(onNewDestinationReceived(uint,Data::Pose)));

    //Basestation WSS --> Map Manager (Waypoint reached)
    connect(baseStationWSS, SIGNAL(sigWaypointReached(uint)),
            mapManager,SLOT(onWaypointReached(uint)));

    //Basestation WSS --> Map Manager (Fault detected)
    connect(baseStationWSS, SIGNAL(sigFaultDetected(QString,uint)),
            mapManager, SLOT(onFaultDetected(QString,uint)));

    //Basestation WSS --> Notification Manager (Fault detected)
    connect(baseStationWSS, SIGNAL(sigFaultDetected(QString,uint)),
            notificationManager, SLOT(onFaultDetected(QString,uint)));

    //Basestation WSS --> Notification Manager (Received new info message)
    connect(baseStationWSS, SIGNAL(sigInfoMessageReceived(Data::InfoMessage,uint)),
            notificationManager, SLOT(onNewInfoMessageReceived(Data::InfoMessage,uint)));

    /*VICTIMS*/
    connect(baseStationWSS, SIGNAL(sigVictimMessage(uint,QImage,double,Data::Pose)),
            notificationManager, SLOT(onVictimDetected(uint,QImage,double,Data::Pose)));

    connect(notificationManager, SIGNAL(signalVictimFound(QList<QImage>,Data::Pose,uint)),
            victimManager, SLOT(onVictimFound(QList<QImage>,Data::Pose,uint)));

    connect(mapManager, SIGNAL(signalVictimAddedByUser(Data::Pose)),
            victimManager, SLOT(onVictimAddedByUser(Data::Pose)));

    connect(victimManager, SIGNAL(signalVictimConfirmed(Data::Pose,uint,uint,bool)),
            mapManager, SLOT(onVictimFound(Data::Pose,uint)));

    connect(victimManager, SIGNAL(signalVictimConfirmed(Data::Pose,uint,uint, bool)),
            baseStationWSS, SLOT(onVictimConfirmed(Data::Pose,uint,uint, bool)));

    connect(victimManager, SIGNAL(signalVictimDeleted(uint,uint)),
            baseStationWSS, SLOT(onVictimDeleted(uint,uint)));

    connect(victimManager, SIGNAL(signalVictimDeleted(uint,uint)),
            mapManager, SLOT(onVictimDeleted(uint)));

    connect(victimManager, SIGNAL(signalVictimSelected(uint)),
            mapManager, SLOT(onVictimSelected(uint)));

    connect(mapManager, SIGNAL(signalVictimSelectedChanged(uint)), victimManager,
            SLOT(onVictimSelected(uint)));

    connect(mapManager, SIGNAL(signalVictimMoved(uint,Data::Pose)),
            victimManager, SLOT(onVictimMoved(uint,Data::Pose)));

    connect(mapManager, SIGNAL(signalVictimMoved(uint,Data::Pose)),
            baseStationWSS, SLOT(onVictimMoved(uint,Data::Pose)));

    connect(baseStationWSS, SIGNAL(sigVictimAssigned(uint,uint,int)),
            victimManager, SLOT(onVictimAssigned(uint,uint,int)));

    /*CONFIGURATIONS*/
    //Basestation Core UI --> Configuration Manager (Show GUI)
    connect(baseStationCoreUI, SIGNAL(siglShowConfigurationGUI()),
            configurationManager, SLOT(onShowModuleActivation()));

    //Configuration Manager --> Basestation WSS (Send Change Module Status Message)
    connect(configurationManager, SIGNAL(sigChangeModuleStatusCFM(QString,bool)),
            baseStationWSS, SLOT(onChangeModuleStatusBW(QString,bool)));

    //Configuration Manager --> Basestation Core (Handle semantic information)
    connect(configurationManager, SIGNAL(sigChangeModuleStatusCFM(QString,bool)),
            this, SLOT(onChangeGlobalModuleStatusBC(QString,bool)));

    //Configuration Manager --> Info Manager
    connect(configurationManager, SIGNAL(sigChangeModuleStatusCFM(QString,bool)),
            singleRobotInfoManager, SLOT(onChangeModuleStatusIM(QString,bool)));

    //Info Manager --> Basestation WSS (Send module status changed)
    connect(singleRobotInfoManager, SIGNAL(sigChangeModuleStatusIM(QString,bool,uint)),
            baseStationWSS, SLOT(onChangeModuleStatusSpecificBW(QString,bool,uint)));

    //Info Manager --> Basestation Core (Handle SLAM)
    connect(singleRobotInfoManager, SIGNAL(sigChangeModuleStatusIM(QString,bool,uint)),
            this, SLOT(onChangeLocalModuleStatusBC(QString,bool,uint)));

    //Configuration Manager --> Basestation WSS (Configure victim detection)
    connect(configurationManager, SIGNAL(signalVictimDetectionConfiguration(double,double,double,double,double,double)),
            baseStationWSS, SLOT(onVictimDetectionConfiguration(double,double,double,double,double,double)));

    connect(configurationManager, SIGNAL(signalMessageRelevanceConfiguration(int,int,int,int)),
            notificationManager, SLOT(onRelevanceConfigurationChanged(int,int,int,int)));

    connect(configurationManager, SIGNAL(signalMinimalPrioritySetted(int)),
            notificationManager, SLOT(onMinimalPriorityUpdated(int)));


    /*SELECTED ROBOT CHANGED HANDLING*/

    //Camera Manager <--> Base Station Core
    connect(cameraManager,SIGNAL(sigSelectedRobotChangedCM(uint)),
            this, SLOT(onSelectedRobotChangedBC(uint)));
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            cameraManager, SLOT(onSelectedRobotChangedCM(uint)));

    //Basestation Core --> Teleoperation Widget
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            teleoperationWidget, SLOT(onSelectedRobotChangedTW(uint)));
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            airTeleoperationWidget, SLOT(onSelectedRobotChangedTW(uint)));

    //Basestation Core <--> Map Manager
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            mapManager, SLOT(onSelectedRobotChangedMM(uint)));
    connect(mapManager,SIGNAL(sigSelectedRobotChangedMM(uint)),
            this,SLOT(onSelectedRobotChangedBC(uint)));

    //Basestation Core <--> Basestation Core UI
    connect(baseStationCoreUI, SIGNAL(sigSelectedRobotChangedBCUM(uint)),
            this, SLOT(onSelectedRobotChangedBC(uint)));
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            baseStationCoreUI, SLOT(onSelectedRobotChanged(uint)));

    //Basestation Core --> Info Manager
    connect(this, SIGNAL(sigSelectedRobotChangedBCM(uint)),
            singleRobotInfoManager, SLOT(onSelectedRobotChangedIM(uint)));

    //Basestation WSS --> Map Manager (Robot reachability)
    connect(baseStationWSS, SIGNAL(sigRobotUnreachableBW(uint)),
            mapManager, SLOT(onRobotUnreachable(uint)));
    connect(baseStationWSS, SIGNAL(sigRobotReachableAgain(uint)),
            mapManager, SLOT(onRobotReachableAgain(uint)));



    //----------------------------Message manager integration in the system-------------------------------------//
    connect(notificationManager, SIGNAL(signalFocusOnRobot(uint)), mapManager, SLOT(onFocusOnRobot(uint)));
    connect(notificationManager, SIGNAL(signalFocusOnRobot(uint)), this, SLOT(onSelectedRobotChangedBC(uint)));
    connect(notificationManager, SIGNAL(signalFocusOnPose(Data::Pose)), mapManager, SLOT(onFocusOnPoint(Data::Pose)));
    connect(singleRobotInfoManager, SIGNAL(signalBrightnessChanged(int,uint)),cameraManager,SLOT(onBrightnessChanged(int,uint)));
    connect(singleRobotInfoManager, SIGNAL(signalContrastChanged(int,uint)),cameraManager,SLOT(onContrastChanged(int,uint)));
    baseStationCoreUI->onSwitchToGlobal();
    this->nActiveRobot = nActiveRobots;
}

void BaseStationCore::onConfigureApplication(uint maxRobots)
{
    Config::robotCount = maxRobots;
    this->baseStationWSS = new BaseStationWSS(0, spawnUi->getBSLocation(), maxRobots);

    //Spawn UI --> Basestation WSS (Modules connection)
    connect(spawnUi, SIGNAL(sigUSARConnect(QString,quint16)),
            baseStationWSS, SLOT(onUSARConnect(QString,quint16)));
    connect(spawnUi, SIGNAL(siglWSSConnect(QString, quint16)),
            baseStationWSS, SLOT(onWSSConnect(QString,quint16)));
    connect(spawnUi, SIGNAL(siglUPISConnect(QString,quint16)),
            baseStationWSS, SLOT(onUPISConnect(QString,quint16)));

    //Spawn UI --> Basestation WSS (Robot spawn)
    connect(spawnUi, SIGNAL(sigSpawnRobotSU(QString,QString,QString,uint,uint,bool,bool)),
            baseStationWSS, SLOT(onSpawnRobotBW(QString,QString,QString,uint,uint,bool,bool)));

    //Spawn UI --> Basestation WSS (Robot respawn)
    connect(spawnUi, SIGNAL(sigRespawnRobot(QString,QString,QString,uint,uint,bool,bool)),
            baseStationWSS, SLOT(killProcess(QString,QString,QString,uint,uint,bool,bool)));

    //Spawn UI --> Teleoperation Widgets (Robot spawn)
    connect(spawnUi, SIGNAL(sigSpawnRobotSU(QString,QString,QString,uint,uint,bool,bool)),
            teleoperationWidget, SLOT(onSpawnRobotTW(QString,QString,QString,uint,uint,bool)));
    connect(spawnUi, SIGNAL(sigSpawnRobotSU(QString,QString,QString,uint,uint,bool,bool)),
            airTeleoperationWidget, SLOT(onSpawnRobotTW(QString,QString,QString,uint,uint,bool)));

    //Spawn UI --> Basestation Core (Robot spawn)
    connect(spawnUi, SIGNAL(sigSpawnRobotSU(QString,QString,QString,uint,uint,bool,bool)),
            this, SLOT(onSpawnRobotBC(QString,QString,QString,uint,uint,bool,bool)));

    //Spawn UI --> Basestation WSS (Disconnect modules)
    connect(spawnUi, SIGNAL(sigDisconnect()),
            baseStationWSS, SLOT(onWSSDisconnect()));

    connect(spawnUi, SIGNAL(sigDisconnect()),
            baseStationWSS, SLOT(onUSARDisconnect()));

    //Spawn UI --> Basestation WSS (Spawn basestation)
    connect(spawnUi, SIGNAL(signalSpawnBS(QString)),
            baseStationWSS, SLOT(onSpawnBS(QString)));

    //Basestation WSS --> Spawn Ui
    connect(baseStationWSS, SIGNAL(sigDisconnectedBW()), spawnUi, SLOT(onDisconnected()));
    connect(baseStationWSS, SIGNAL(sigConnectedBW()), spawnUi, SLOT(onModuleConnected()));
    connect(baseStationWSS, SIGNAL(sigErrorBW(QString)), spawnUi, SLOT(appendOnConsole(QString)));
    connect(baseStationWSS, SIGNAL(sigInitialLocation(QString)), spawnUi, SLOT(onInitialLocation(QString)));

    //TODO: connection for sendRawMessage from the spawnUI to connectionController.
    connect(spawnUi, SIGNAL(signalRawMessage(Data::Message)), baseStationWSS, SLOT(onSendRawCommand(Data::Message)));

    /*Camera manager*/
    cameraManager->configureThumbnails(maxRobots);
    connect(baseStationWSS, SIGNAL(sigRobotUnreachableBW(uint)),
            cameraManager, SLOT(onRobotUnreachable(uint)));
}

void BaseStationCore::onSelectedRobotChangedBC(uint robotID)
{
    if(robotID != this->selectedRobotID)
    {
        baseStationWSS->onFramerateSetting(this->selectedRobotID, LOW_FRAMERATE);
        baseStationWSS->onFramerateSetting(robotID, HIGH_FRAMERATE);
        this->selectedRobotID = robotID;
        emit sigSelectedRobotChangedBCM(robotID);
    }
}

void BaseStationCore::onChangeGlobalModuleStatusBC(QString module, bool status)
{
    if(module == SEMANTIC_MAPPING && semanticMappingManager != 0){
        semanticMappingManager->changeStatus(status);
    }
}

void BaseStationCore::onChangeLocalModuleStatusBC(const QString module, bool status, uint robotid)
{
    if(module == POARET_SLAM_MODULE && !status) {
        mapManager->changeStatus(robotid);
    }
}

void BaseStationCore::onSpawnRobotBC(const QString, const QString, const QString,
                                     const uint nActiveRobot, uint, bool, bool kenaf)
{
    if(kenaf)
    {
        kenafs->append(nActiveRobot);
        teleoperationWidget->addKenaf(nActiveRobot);
        if(mapManager != 0)
            mapManager->addKenaf(nActiveRobot);

    }
}

}
