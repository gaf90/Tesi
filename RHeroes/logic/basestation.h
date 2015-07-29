#ifndef BASESTATION_H
#define BASESTATION_H

#include <QObject>
#include <QStack>
#include <QProcess>
#include "connection/usarcontroller.h"
#include "connection/wsscontroller.h"
#include "graphics/basestationui.h"
#include "data/message.h"
#include "middleware/driver/wirelessdriver.h"
#include "robot.h"

namespace Logic{

/**
 * This class represents the controller for the BaseStation.
 * It is responsible for the communication between the user and USARSim.
 */
class BaseStation : public QObject
{
    Q_OBJECT
public:
    /**
     * Default Constructor.
     */
    explicit BaseStation(QObject *parent = 0);
    /**
     * Virtual Destructor.
     */
    virtual ~BaseStation();

    void onProcReadyRead();

signals:
    /**
     * Signal emitted when a new initial location for the
     * robots is available.
     * @param param the new available location.
     */
    void signalInitialLocation(const QString &params);

public slots:
    void onProcessReadyRead();
    /**
     * Slot invoked when the UI signals to connect to USARSim.
     * @param usarAddress the IP address of USARSim.
     * @param usarPort the port of USARSim.
     */
    void onUSARConnect(const QString &usarAddress, const quint16 usarPort);
    /**
     * Slot invoked when the UI signals to connect to UPIS.
     * This slot only stores the arguments to further spawns of the robots.
     * @param upisAddress the IP address of UPIS.
     * @param upisPort the port of UPIS.
     */
    void onUPISConnect(const QString &upisAddress, const quint16 upisPort);
    /**
     * Slot invoked when the UI signals to connect to WSS.
     * This slot only stores the arguments to further spawns of the robots.
     * @param wssAddress the IP address of WSS.
     * @param wssPort the port of WSS.
     */
    void onWSSConnect(const QString &wssAddress, const quint16 wssPort);
    /**
     * Slot invoked when the UI signals to spawn a new robot.
     * The method spawns a robot at the location passed as argument and
     * gives to it the informations to connect to USARSim and UPIS.
     * @param location the location in which the robot must be spawned.
     */
    void onSpawnRobot(const QString &location);
    /**
     * Slot invoked when the controllers have installed succesfully a connection.
     */
    void onConnected();
    /**
     * Slot invoked for logging purposes, up to now.
     * It logs the messages received from the controllers.
     */
    void onMessage(const Data::Message &message);

    /**
     * Slot invoked when a robot UI has to communicate
     * a message to the corresponding robot in USARSim.
     * @param message the Message to communicate.
     */
    void onRobotMessage(const Data::Message &message, uint robotId);

    void onProcessError(QProcess::ProcessError error);

public:
    uint getRobotId();

private:
   Connection::USARController* usarController;
   Connection::WSSController* wssController;
   Middleware::WirelessDriver* wirelessDriver;
   uint nActiveRobot;
   BaseStationUi* graphics;
   QStack<RobotUi *> *spawnedRobots;
   QStack<QProcess *> *spawnedProcess;


   QString usarAddress, upisAddress, wssAddress;
   uint usarPort, upisPort, wssPort;

};

}

#endif // BASESTATION_H
