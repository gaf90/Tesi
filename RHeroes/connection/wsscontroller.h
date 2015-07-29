#ifndef WSSCONTROLLER_H
#define WSSCONTROLLER_H

#include "abstractsocketcontroller.h"
#include "data/wssmessage.h"
#include "data/wirelessmessage.h"
#include "connection/rountingcontroller.h"
#include <QHash>
#include <QSignalMapper>
#include <QByteArray>
#include <QTcpServer>
#include <QTimer>

namespace Connection {

class WSSController : public AbstractSocketController
{
    Q_OBJECT

public:
    explicit WSSController(uint nRobot, QObject *parent = 0);
    virtual ~WSSController();


signals:
    void sigMessage(const Data::Message &msg);
    void sigSendDistanceVector(const Data::Message &msg);
    void sigRobotNotReachable();
    void sigInitializeConnection(const QString& robot);
    void sigUpdateSignalStrength(const QString& strength);

public slots:
    void sendMessage(const Data::Message &msg);
    void baseStationHandshake();
    void onDistanceVector(const QString &sender, const QHash<QString, int> &connections);

protected slots:
    /**
     * AbstractSocketController::invokeReadData() implementation for
     * WSSController
     */
    void invokeReadData();

private slots:
    void serverConnection();
    void robotDisconnected(const QString &name);
    void robotData(const QString &name);
    void wssConnected();
    void initDataExchange(const QString &dest);

    //Routing
    void checkForConnectedPeers();
    void sendDistanceVector(QHash<QString, int> dv);
    void onNewRoutingConnection(QString peer);

    void debugCore();
    void checkForConnetionErrors();

private:
    int nRobot;
    QTcpServer *server;
    QHash<quint16, QTcpSocket *> pendingReverseDNS;
    QHash<QString, QTcpSocket *> connectionCache;
    QHash<QString, QByteArray> queuedData;
    QSignalMapper *mapperConnected, *mapperDisconnected, *mapperData;
    RoutingController *router;
    QTimer *connectionErrorTimer;
    void forceClose(QString robot);
    //Routing


    void commenceDataExchange(const Data::WSSMessage &msg);
    void socketCaching(const QString &dest, QTcpSocket *sock);
    int sendData(const QString &dest, const QByteArray &data);

    //Routing:
//    void sendDistanceVector();
};

}

#endif // WSSCONTROLLER_H
