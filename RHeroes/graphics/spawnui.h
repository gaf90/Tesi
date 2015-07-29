#ifndef SPAWNUI_H
#define SPAWNUI_H

#include <QMainWindow>
#include <QErrorMessage>

#include "data/message.h"

namespace Ui {
    class SpawnUI;
}

namespace graphics{

/**
* @brief UI for the startup of the systems
*
* This basic UI allows the supervisor to easily set up the appliation
* and spawn both the baseStation and the robots. The useful parameters retrieved
* after the startup process can be used by BaseStationCore.
*
*/
class SpawnUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit SpawnUI(QWidget *parent = 0);
    ~SpawnUI();

    QString getBSLocation();

signals:

    /**
    * This signal is emitted when the initial spawn process has been completed.
    * @param all the initial information needed by the global, main interface!!!
    */
    void sigApplicationStarted(uint nActiveRobots);

    /**
    * This signal is emitted when the initial configuration process has been completed.
    * @param all the initial information needed by the global, main interface!!!
    */
    void sigApplicationConfigure(uint maxRobots);

    /**
     * Signal emitted to notify the UPIS address-port pair.
     * @param address the UPIS IP address.
     * @param port the UPIS port number.
     */
    void siglUPISConnect(const QString &address, const quint16 port);
    /**
     * Signal emitted to connect to USAR,
     * running at the address-port pair in the parameters.
     * @param address the USAR IP address.
     * @param port the USAR port number.
     */
    void sigUSARConnect(const QString &address, const quint16 port);

    /**
     * Signal emitted to connect to WSS,
     * running at the address-port pair in the parameters.
     * @param address the WSS IP address.
     * @param port the WSS port number.
     */
    void siglWSSConnect(const QString &address, const quint16 port);
    /**
     * Signal emitted to notify that a robot must be spawned.
     * @param location the location in the map where the robot must be spawned.
     */
    void sigSpawnRobotSU(const QString &location, const QString &rotation, const QString &bsLocation,
                          const uint nActiveRobot, uint maxRobots, bool aerial, bool kenaf);
    /**
     * Signal emitted when the user want to disconnect from USAR.
     */
    void sigDisconnect();
    /**
    * Slot used to send a raw command to USARSim, by mean of the spawn UI.Mainly used for
    * debugging or moving the base station.
    */
    void signalRawMessage(Data::Message);

    void signalSpawnBS(QString);

    void sigRespawnRobot(const QString &location, const QString &rotation, const QString &bsLocation,
                            const uint nActiveRobot, uint maxRobots, bool aerial, bool kenaf);

public slots:



/*--------------------------------------------------------------------------------------------------
                    Slots for the management of the UI behaviour
--------------------------------------------------------------------------------------------------*/


    /**
     * Slot invoked when the controllers have installed succesfully a connection.
     */
    void onModuleConnected();

    /**
     * Slot invoked when the UI receives the Disconnected()
     * signal from USAR.
     */
    void onDisconnected();

    /**
     * Slot invoked to update the list of available initial location.
     * @param location a new available location.
     */
    void onInitialLocation(const QString &location);

    /**
    * Slot used to show the coordinates corresponding to a starting pose!
    */
    void onInitialLocationSelected(QString loc);

    /**
     * Slot invoked when the "Connect" button is pressed.
     * It installs a connection with USARSim.
     */
    void onConnectButtonClicked();
    /**
     * Slot invoked when the "Send Raw" button is pressed.
     * It sends to USARSim a command.
     */
    void onSendRawButtonClicked();
    /**
     * Slot invoked when the "Spawn" button is pressed.
     * It informs the user interface that the process for
     * spawning a new robot must be started.
     */
    void onSpawnButtonClicked();
    /**
     * Slot invoked when the "AirRobot!" button is pressed.
     * It informs the user interface that the process for
     * spawning a new air robot must be started.
     */
    void onAirSpawnButtonClicked();

    void onKenafSpawnButtonClicked();

    /**
     * Slot invoked to log the messages received from the servers;
     * @param message a QString to log.
     */
    void appendOnConsole(const QString &message);

    /**
     * Slot invoked to log the messages received from the servers;
     * @param message a Message to log.
     */
    void appendOnConsole(const Data::Message &message);

    /**
    * This slot handles the starting of the global UI
    */
    void onStartApplication();

    /**
    * This slot handles the configuration of the application.
    */
    void onConfigureButtonClicked();

    void onSpawnBS();

    void onRespawnButtonClicked();

private:
    void enableComponents(bool enable);

    QString usarAddress, upisAddress, wssAddress;
    uint usarPort, upisPort, wssPort;
    uint nActiveRobot;

    Ui::SpawnUI *ui;
    QHash<QString,QStringList> *locations;
    QErrorMessage *error;

    bool bsSpawned;

    int robotToRespawn;

    typedef struct{
        QString location, rotation, bsLocation;
        uint nActiveRobot, maxRobots;
        bool aerial, kenaf;
    } t_spawnInfo;

    QHash<uint, t_spawnInfo> *spawnedRobots;
};

}
#endif // SPAWNUI_H
