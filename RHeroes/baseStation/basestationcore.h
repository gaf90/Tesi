#ifndef BASESTATION_H
#define BASESTATION_H

#define HIGH_FRAMERATE true
#define LOW_FRAMERATE false

#include <QObject>
#include "baseStation/mapmanager.h"
#include "baseStation/cameramanager.h"
#include "baseStation/infomanager.h"
#include "graphics/basestationcoreui.h"
#include "baseStation/basestationwss.h"
#include "graphics/spawnui.h"
#include "shared/config.h"
#include "baseStation/notificationmanager.h"
#include "victimmanager.h"
#include "baseStation/configurationmanager.h"

#include "semanticMapping/semanticmappingmodule.h"
#include "semanticMapping/test/displayroomtest.h"

namespace BaseStation{

/**
* @brief main class of the base station logic
*
* This class is used to connect signals and slots of the single logic modules on the
* base station. It contains also the procedures to switch between different interfaces
* (teleoperation, global and spawn interfaces).
*
* @see
*
* @author Alain Caltieri
*/
class BaseStationCore : public QObject
{
    Q_OBJECT
public:
    explicit BaseStationCore(QObject *parent = 0);

    virtual ~BaseStationCore();

signals:

    void signalSwitchToTeleoperation();
    void signalSwitchToGlobal();
    void sigSelectedRobotChangedBCM(uint robotID);

private slots:

    /**
    * This method is used to manage the switching process to the Teleoperation UI.
    */
    void onSwitchToTeleoperation();

    /**
    * This method is used to manage the switching process to the Global UI.
    */
    void onSwitchToGlobal();

    /**
    * This method is called when the spawn process has been completed.
    * It receives all global parameters needed by the applocation and its modules
    * that have been setted in the initial spawnUI.
    */
    void onStartedApplication(uint nActiveRobots);

    /**
    * Handles the very first phase of the deployment of the system, receiving global parameters.
    */
    void onConfigureApplication(uint maxRobots);

    /**
    * This slot handles incoming signals from single modules that invoke the change of the actual
    * selected robot.
    */
    void onSelectedRobotChangedBC(uint robotID);

    void onChangeGlobalModuleStatusBC(QString module, bool activate);

    void onSpawnRobotBC(const QString, const QString, const QString,
                        const uint nActiveRobot, uint , bool , bool kenaf);

    void onChangeLocalModuleStatusBC(const QString, bool, uint);


private:
    //single grafical modules
    MapManager *mapManager;
    CameraManager * cameraManager;
    NotificationManager *notificationManager;
    InfoManager *singleRobotInfoManager;
    VictimManager *victimManager;
    ConfigurationManager *configurationManager;

    //communication
    BaseStationWSS *baseStationWSS;

    //Interfaces
    graphics::BaseStationCoreUi *baseStationCoreUI;
    graphics::SpawnUI *spawnUi;

    //max deployable robots.
    uint nActiveRobot;

    uint selectedRobotID;

    SemanticMapping::SemanticMappingModule *semanticMappingManager;
    SemanticMapping::Test::DisplayRoomTest *semanticMappingTestRoom;

    graphics::TeleoperationWidget *teleoperationWidget;
    graphics::AirTeleoperationWidget *airTeleoperationWidget;
    QList<uint> *kenafs;
};


}
#endif // BASESTATION_H
