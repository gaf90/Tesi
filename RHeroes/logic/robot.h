#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QList>
#include <QFile>
#include "logic/robotcontroller.h"
#include "middleware/driver/driver.h"
#include "middleware/driver/wirelessdriver.h"
#include "middleware/sensor/sensor.h"
#include "connection/usarcontroller.h"
#include "connection/upiscontroller.h"
#include "connection/wsscontroller.h"
#include "graphics/robotui.h"
#include "semanticMapping/semanticmappingmodule.h"
#include "data/waypointcommand.h"
#include "slam/slammodule.h"
#include "victimDetection/victimdetectionmodule.h"
#include "exploration/explorationmodule.h"
#include "coordination/coordinationmodule.h"
#include "data/robotstate.h"
#include "shared/utilities.h"
#include "pathPlanner/pathplannermodule.h"
#include "data/highlevelcommand.h"

//#define SPEED_CALIBRATION

class RobotController;

/**
 * This class represents the high-level robot.
 * It connects the sensors and the drivers to
 * the controllers that are connected to the servers
 */
class Robot : public QObject
{
    Q_OBJECT
public:

    enum movementStateEnum {FRONT,RIGHT,LEFT,BACK};

    /**
     * Contructor for the robot.
     */
    explicit Robot(const QString &location, const QString &rotation, const uint identifier, bool isKenaf,
                   QObject *parent = 0);
    /**
     * Destructor for the robot.
     */
    virtual ~Robot();

    /**
     * Method that spawns a robot in the simulation, connecting it
     * to USARSim and to UPIS.
     * @param usarAddress the IP address of USARSim.
     * @param usarPort the port of USARSim.
     * @param upisAddress the IP address of UPIS.
     * @param upisPort the port of UPIS.
     */
    void spawnRobot();

    /**
     * This method allow the BaseStation to get the GUI of
     * the spawned robots.
     * @return the GUI of the robot.
     *
    const RobotUi * getRobotUi() const;*/   

    void handleModuleActivationMessage(const Data::BuddyMessage *buddy);
    void handleWaypointMessage(const Data::BuddyMessage *buddy);
    void handleVictimDetectionConf(const Data::BuddyMessage *buddy);
    void handleInfoMessage(const Data::InfoMessage *infoData);
    void initSlamModule(const QString &location, const QString &rotation, const uint identifier);

    void initSensorsAndDrivers(const uint identifier);
    void handleUsarControllerSignals();
    void handleUpisControllerSignals();
    void handleWSSControllerSignals();
    void handleWirelessDriverSignals();
signals:
    void sigWaypointReceived(const Data::WaypointCommand *waypoint);
    void signalVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                            double hMax, double sMax, double vMax);
    void sigEnableUserCriteria(bool activate , const Data::HighLevelCommand *command);

public slots:
    void onPowerSignalDataGathered();


private slots:
    /**
     * Slot invoked when the USARSim controller has installed succesfully a connection.
     */
    void onUSARSimConnected();
    void onCoordinationMessage(const Data::Message &msg);

    void onUpdateSignalStrength(QString strength);

//    /**
//     * Slot invoked when the WSS controller has installed successfully a connection.
//     */
//    void onWSSConnected();
    /**
     * Slot invoked when the controllers have succesfully disconnected from the servers.
     */
    void onUSARSimDisconnected();

    /**
      * Slot invoked to handle the 'sensor data' frome the BaseStation.
      * @param msg the message that contains
      */
    void onSensorData(const Data::Message &msg);

    void handlePlasterMapper();

    void onUPISSimConnected();

    void onUPISDisconnected();

    void onUPISError(QString error);

#ifdef SPEED_CALIBRATION
    void sendSpeed();
    void saveGroundTruth();
private:
    Data::Pose groundTruth;
    double vv, ii;
    FILE *ff;
#endif

private:

    void changeModuleStatus(const QString &moduleName, bool enable);

    RobotController* robotController;
    Connection::USARController *usarController;
    Connection::UPISController *upisController;
    Connection::WSSController *wssController;
    QList<Middleware::Sensor *> *sensors;
    QList<Middleware::Driver *> *drivers;
    RobotUi* graphics;
    Middleware::WirelessDriver *wirelessDriver;

    VictimDetection::VictimDetectionModule * victimModule;
    SLAM::SLAMModule *slamModule;
    SemanticMapping::SemanticMappingModule * semMapModule;
    Exploration::ExplorationModule * explorationModule;
    Coordination::CoordinationModule * coordinationModule;
    PathPlanner::PathPlannerModule * pathPlannerModule;
    QTimer plasterMapper;

    QString initialLocation;
    QString initialRotation;
    uint identifier;

    bool isKenaf;
};

#endif // ROBOT_H
