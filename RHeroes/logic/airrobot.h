#ifndef AIRROBOT_H
#define AIRROBOT_H

#include <QObject>
#include <QList>
#include "logic/robotcontroller.h"
#include "middleware/driver/driver.h"
#include "middleware/driver/wirelessdriver.h"
#include "middleware/sensor/sensor.h"
#include "connection/usarcontroller.h"
#include "connection/upiscontroller.h"
#include "connection/wsscontroller.h"
#include "shared/utilities.h"
#include "middleware/driver/aircontroldriver.h"
//#include "data/robotstate.h" Useful???

/**
 * This class represents the high-level air-robot.
 * It connects the sensors and the drivers to
 * the controllers that are connected to the servers
 */
class AirRobot : public QObject
{
    Q_OBJECT
public:
    /**
     * Contructor for the air-robot.
     */
    explicit AirRobot(const QString &location, const QString &rotation,
                      const uint identifier, QObject *parent = 0);

    /**
     * Destructor for the air-robot.
     */
    virtual ~AirRobot();

    /**
     * Method that spawns a robot in the simulation, connecting it
     * to USARSim and to UPIS.
     * @param usarAddress the IP address of USARSim.
     * @param usarPort the port of USARSim.
     * @param upisAddress the IP address of UPIS.
     * @param upisPort the port of UPIS.
     */
    void spawnRobot();


signals:

public slots:
        void onPowerSignalDataGathered();

private slots:
    /**
     * Slot invoked when the USARSim controller has installed succesfully a connection.
     */
    void onUSARSimConnected();

    /**
     * Slot invoked when the controllers have succesfully disconnected from the servers.
     */
    void onUSARSimDisconnected();

    /**
      * Slot invoked to handle the 'sensor data' frome the BaseStation.
      * @param msg the message that contains
      */
    void onSensorData(const Data::Message &msg);

    void onUPISSimConnected();

    void onUPISDisconnected();

    void onUPISError(QString error);

private:
    RobotController* robotController;
    Connection::USARController *usarController;
    Connection::UPISController *upisController;
    Connection::WSSController *wssController;
    QList<Middleware::Sensor *> *sensors;
    QList<Middleware::Driver *> *drivers;
    Middleware::WirelessDriver *wirelessDriver;

    Middleware::AirControlDriver *airDriver;

    QString initialLocation;
    QString initialRotation;
    uint identifier;

//OLD useless things-----------------------------------------------------------------

//public slots:
//    void onSLAMNewPose(SLAM::TimedPose pose);


//private slots:
//    void onCoordinationMessage(const Data::Message &msg);
//    void handlePlasterMapper();

//private:

//    RobotUi* graphics;
//    SLAM::SLAMModule *slamModule;
//    SemanticMapping::SemanticMappingModule * semMapModule;
//    Exploration::ExplorationModule * expModule;
//    Coordination::CoordinationModule * coordModule;
//    PathPlanner::PathPlannerModule * pathPlannerModule;
//    QTimer plasterMapper;
};

#endif // AIRROBOT_H
