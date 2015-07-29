#ifndef BASESTATIONUI_H
#define BASESTATIONUI_H

#include <QMainWindow>
#include "data/message.h"

#define CONNECT "Connect"
#define DISCONNECT "Disconnect"

namespace Ui {
    class BaseStationUi;
}

/**
 * This class represents the user interface for the
 * robots' operator. It allows to create new robots
 * and it collects the shared information among them.
 */
class BaseStationUi : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * Constructor for the BaseStation.
     */
    explicit BaseStationUi(QWidget *parent = 0);
    /**
     * Destructor for the BaseStation.
     */
    virtual ~BaseStationUi();

private:
    void enableComponents(bool enable);

signals:

    /**
     * Signal emitted when the UI must send a Raw Command to USARSim.
     * @param msg the message to send to USAR.
     */
    void sigMessage(const Data::Message &msg);
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
    void sigSpawnRobotBU(const QString &location);
    /**
     * Signal emitted when the user want to disconnect from USAR.
     */
    void sigDisconnect();

private slots:
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
     * Slot invoked when the UI receives the Connected()
     * signal from USAR.
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

private:
    Ui::BaseStationUi *ui;
};

#endif // BASESTATIONUI_H
