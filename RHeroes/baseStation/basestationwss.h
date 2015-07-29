#ifndef BASESTATIONWSS_H
#define BASESTATIONWSS_H

#define BS_WSS_INACTIVITY_TIMEOUT 5000

#include <QObject>
#include <QStack>
#include <QProcess>

#include "data/mapmessage.h"
#include "data/semanticmapinfomessage.h"
#include "data/victimmessage.h"
#include "data/cameradata.h"
#include "data/highlevelcommand.h"
#include "data/moduleactivationmessage.h"
#include "data/waypointcommand.h"
#include "data/infomessage.h"

#include "slam/map.h"

#include "connection/usarcontroller.h"
#include "connection/wsscontroller.h"

#include "middleware/driver/wirelessdriver.h"

namespace BaseStation{

/**
* @brief BaseStationWSS handles transmissione from the Basesstation to both USARSim and single robots
*
* BaseStationWSS handles every incoming message from both single robots and USARSim, and
* sends messages through WSS network to single robots (for commands, information requests, ect)
*
*/
class BaseStationWSS : public QObject
{
    Q_OBJECT
public:
    explicit BaseStationWSS(QObject *parent = 0, QString bsLocation = "0,0,0", int maxNRobots = 0);

    /**
    * destroys the BaseStationWSS element.
    */
    virtual ~BaseStationWSS();


signals:
/*------------------------------------------------------------------------------------
                      Signals to notify incoming information
------------------------------------------------------------------------------------*/
/*
        Usiamo i segnali per smistare le informazioni ai vari moduli, mantenendo la
        struttura dei messaggi usata dai singoli robot. TODO
*/

    /**
    * This signal is emitted when a well formed message about victim detection has been received.
    * @param robotID the id of the robot that sent the message
    * @param image a QImage frame that contains the possible victim
    * @param confidence the confidence, calculated by the victim detection module, that the image
    * contains really a victim
    * @param position the estimated position of the victim.
    */
    void sigVictimMessage(uint robotID, const QImage &image, double confidence, const Data::Pose position);

    void sigVictimAssigned(uint victimID, uint robotID, int timeToReach);

    void sigNewMapDataReceived(const SLAM::Map &map, uint robotID);

    /**
    * Signal a new frame is arrived from a robot via wss
    * @param robotID the robot that has sent the frame
    * @param image the frame arrived
    */
    void sigCameraDataReceived(const uint robotID, const QImage &image);

    /**
    * This signal is emitted when new information arrives about a robot.
    */
    void sigInfoMessageReceived(const Data::InfoMessage message, uint robotID);

    /**
    * This signal is emitted when a waypoint has been reached by a robot (according to what the
    * robot says...)
    * @param RobotID the Id of the involved robot
    */
    void sigWaypointReached(uint robotID);

    void sigNewDestinationReceived(uint robotID, Data::Pose position);

    void sigFaultDetected(QString message, uint robotID);

/*-----------------------------------------------------------------------------------
                        SPAWNUI communication messages.
        - This handles the system setup and robots' spawning processes -
-----------------------------------------------------------------------------------*/

    /**
     * Signal emitted when a new initial location for the
     * robots is available.
     * @param param the new available location.
     */
    void sigInitialLocation(const QString &params);

/*------------------------------------------------------------------------------------
                           USARSim and WSS notification signals
------------------------------------------------------------------------------------*/
    /**
    * signal emitted when the basestation has been connected
    */
    void sigDisconnectedBW();

    /**
    * signal emitted when the basestation has been connected
    */
    void sigConnectedBW();


    void sigErrorBW(const QString);

    /**
    * This signal is emitted when it is found a robot, not spawned by the UI, that communicates
    * with the base station. Basically for debugging purposes!
    * @param robotID the ID of the robot that connected to the BS!
    */
    void sigIncomingRobotConnectionBW(uint robotID);

    /**
    * This is emitted when a robot becames unreachable (it hasn't sent any message for
    * BS_WSS_INACTIVITY_TIMEOUT milliseconds).
    */
    void sigRobotUnreachableBW(uint robotID);

    /**
    * This is emitted when a robot, previously unreachable, becames newly reachable. A robot
    * becomes again reachable when it sends to the base station a message.
    */
    void sigRobotReachableAgain(uint robotID);

public slots:
/*--------------------------------------------------------------------------------------------------
                        Slots used to manage the connection establishment
--------------------------------------------------------------------------------------------------*/

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
        * Slot invoked to disconnect from the simulator.
        */
        void onUSARDisconnect();

        /**
        * Slot invoked to disconnect from the wss server.
        */
        void onWSSDisconnect();

        /**
        * connects to wss when usar has been connected.
        */
        void onConnected();

        /**
        * Handles the disconnected signal from the usarController
        */
        void onDisconnected();

        /**
        * Richiede le startposes, dopo aver atteso per sicurezza un pochino...
        */
        void onConnectedAndWaited();

        /**
        handles errors from both usar and wss controllers
        */
        void onError(QString error);

        /**
         * Slot invoked when the UI signals to spawn a new robot.
         * The method spawns a robot at the location passed as argument and
         * gives to it the informations to connect to USARSim and UPIS.
         * @param location the location in which the robot must be spawned.
         */
        void onSpawnRobotBW(const QString &location, const QString &rotation,
                          const QString &bsLocation, uint nActiveRobot, uint maxRobots, bool aerial, bool isKenaf = false);

        /**
        * slot used for transmit a message to USARSim through the usarController,
        * only for debug or emergencies.
        */
        void onSendRawCommand(Data::Message mex);

        void onSpawnBS(QString location);

        void dummySlot(){
            wssController->connectToHost(wssAddress, wssPort);
        }

/*-----------------------------------------------------------------------------------------------------------
                        Methods used to communicate with single robots
-----------------------------------------------------------------------------------------------------------*/

/*----------------------Incoming messages-----------------------------*/
    /**
     * This slot receives all incoming messages from both USARSim and WSS. It analyzes such messages,
     * parse them in a proper format, then emits a signal that must be captured by the module that will
     * handle the message.
     * @param message is the received message from the wssController or usarController
     */
    void onMessage(const Data::Message &message);


/*---------------------------Commands----------------------------------*/
    /**
    * This slot handles the sending of a High level command to a single robot. It sends the message through
    * WSS to the related robot, properly including it into a BUDDYMESSAGE
    * @param pose the Pose that specifies the direction to be explored or the area to be serched
    * @param notifyWhenFinished if true the robot must notify to the base station when the task is completed
    * @param waitTime specifies how much time the robot must wait for human intervention after completing the task,
    * useful only if notifyWhenFinished=true
    * @param robotID the ID of the robot that will receive the message
    */
    void onHighLevelCommand(double x, double y, bool notifyWhenFinished, uint waitTime, uint robotID, bool isDirection);

    /**
    * This slot handles the sending of a waypoint command to a single robot. It sends the message through
    * WSS to the related robot, properly including it into a BUDDYMESSAGE
    * @param waypoints the QList of poses that the robot must reach when executing the command
    * @param notifyWhenFinished if true the robot must notify to the base station when the task is completed
    * @param waitTime specifies how much time the robot must wait for human intervention after completing the task,
    * useful only if notifyWhenFinished=true
    * @param robotID the ID of the robot that will receive the message
    */
    void onWaypointCommand(Data::Pose waypoint, bool notifyWhenFinished, uint waitTime, uint robotID);

    /**
    * This slot handles the sending of a waypoint command to a single robot. It sends the message through
    * WSS to the related robot, properly including it into a BUDDYMESSAGE. Used for ground robots only
    * @param sLeft is the speed of the left wheel of the robot
    * @param sRight is the speed of the right wheel of the robot
    * @param robotID the ID of the robot that will receive the message
    */
    void onSendTeleoperationCommand(double sLeft, double sRight, uint RobotID);

    /**
    * This slot handles the sending of a air teleoperation command to a single robot. It sends the message through
    * WSS to the related robot, properly including it into a BUDDYMESSAGE. Used for air robots only
    * @param altitude is the speed for up/down movements
    * @param linear is the speed for forward/backward movements
    * @param lateral is the speed for shift movements
    * @param rotational is the speed for rotational movements
    * @param robotID the ID of the robot that will receive the message
    */
    void onSendAirTeleoperationCommand(double altitude, double linear, double lateral,
                                   double rotational, uint robotID);

    /**
    * This slot handles the sending of a framerate setting command to the single robot. It is used
    * to set the framerate of the camera views: if the camera data is used for the focus view
    * it must have an higher frameRate than if it is used in thumbnails.
    * @param robot the robot's id
    * @param highFrameRate a boolean that specifies if the framerate must be set to high (true) or low
    */
    void onFramerateSetting(uint robot, bool highFrameRate);

    /**
    * This slot handles the request for single robot informations. wssController will then submit the
    * request to the robot involved
    * @param robotID the id of the robot who's information is requested
    */
    void onSendInfoMessage(uint robotID);

    /**
    * This slot handles the victim confirmation message and the victim re-position one (isNew == false)
    */
    void onVictimConfirmed(Data::Pose position, uint victimID, uint robotID, bool isNew);

    /**
    * This slot is used to delete a victim from the handled ones!
    */
    void onVictimDeleted(uint victimID, uint robotID);

    void onVictimMoved(uint id, const Data::Pose &position);

    void onVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                        double hMax, double sMax, double vMax);

/*--------------------other messages-----------------------------*/
    /**
     * Slot invoked when a robot UI has to communicate
     * a message to the corresponding robot in USARSim.
     * @param message the Message to communicate.
     */
    void onRobotMessage(const Data::Message &message, uint robotId);

    /**
    * This slot handles activation/deactivation requests os single modules for all robots.
    * The message must be delivered affidably to all robots.
    */ //TODO after routing implementation
    void onChangeModuleStatusBW(const QString &module, bool enable);

    void onChangeModuleStatusSpecificBW(const QString &module, bool enable, uint robotID);
    //    void onInfoRequest();
    //    void onSemanticInfoAdded();

    void onMapNotWorking(uint robot);

/*-----------------------------------------------------------------------------------------------------------
                                Handling of reachability of robots
-----------------------------------------------------------------------------------------------------------*/
    void reachabilityCheck();

    void killProcess(const QString &location, const QString &rotation, const QString &bsLocation, uint nActiveRobot,
                     uint maxRobots, bool aerial, bool kenaf);

    void respawnProcess();


private:

    Connection::USARController* usarController;
    Connection::WSSController* wssController;
    Middleware::WirelessDriver* wirelessDriver;

    QHash<uint, QProcess *> *spawnedProcess;

    QString usarAddress, upisAddress, wssAddress;
    uint usarPort, upisPort, wssPort;
    QString bsLocation;

    QList<uint> *reachableRobots;
    QList<uint> *unreachableRobots;
    QTimer *reacheabilityTimer;

    int maxRobots;

    //for respawningconst
    QString respawnLocation, respawnRotation;
    uint robotToRespawn;
    bool respawnAerial, respawnKenaf;
};



}
#endif // BASESTATIONWSS_H
