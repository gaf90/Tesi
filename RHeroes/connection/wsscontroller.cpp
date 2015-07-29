#include "wsscontroller.h"
#include "data/buddymessage.h"
#include "shared/utilities.h"
#include "shared/config.h"
#include <typeinfo>
#ifdef __DEBUG__
#   include "data/buddymessage.h"
#endif

#define TIME_TO_RECONNECT 3000

namespace Connection {

using namespace Data;

WSSController::WSSController(uint nRobot, QObject *parent) :
    AbstractSocketController(parent), nRobot(nRobot)
{
    mapperConnected     = new QSignalMapper(this);
    mapperDisconnected  = new QSignalMapper(this);
    mapperData          = new QSignalMapper(this);
    server              = new QTcpServer();
    connect(this, SIGNAL(sigConnected()), this, SLOT(wssConnected()));
    connect(this, SIGNAL(sigDisconnected()),this,SLOT(reconnectToHost()));
    connect(server, SIGNAL(newConnection()), this, SLOT(serverConnection()));

    router = new RoutingController(0, nRobot);
    connect(router, SIGNAL(signalRoutingUpdated(QHash<QString,int>)), this, SLOT(sendDistanceVector(QHash<QString,int>)));
    connect(router, SIGNAL(signalNewReachablePeer(QString)), this, SLOT(onNewRoutingConnection(QString)));
    connect(router, SIGNAL(signalNewConnectedPeer(QString)), this, SLOT(initDataExchange(QString)));
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(checkForConnectedPeers()));
    timer->start(SIGNAL_STRENGTH_REQUEST_TIMER);
    this->checkForConnectedPeers();

    QTimer *debugTimer = new QTimer(this);
    connect(debugTimer, SIGNAL(timeout()), this, SLOT(debugCore()));
    debugTimer->start(3000);

    connectionErrorTimer = new QTimer(this);

    connect(mapperDisconnected, SIGNAL(mapped(const QString &)),
            this, SLOT(robotDisconnected(const QString &)));
    connect(mapperData, SIGNAL(mapped(const QString &)),
            this, SLOT(robotData(const QString &)));
}

WSSController::~WSSController()
{
    delete mapperConnected;
    delete mapperDisconnected;
    delete mapperData;
    foreach(QString key, connectionCache.keys()) {
        connectionCache[key]->disconnectFromHost();
        connectionCache[key]->close();
        delete connectionCache[key];
    }
}

void WSSController::onDistanceVector(const QString &sender, const QHash<QString, int> &connections)
{
    ldbg << "Receive Distance vector" << sender.toAscii().data() << endl;
    if(sender != robotNameFromIndex(nRobot))
        router->processReceivedDistanceVector(sender, connections);
}

void WSSController::sendMessage(const Message &msg)
{
    //static QMutex m;
    //m.lock();

    if(typeid(msg) == typeid(const WSSMessage &)) {
        if(socket->state() == socket->ConnectedState) {
            const WSSMessage &wssmsg = (const WSSMessage &) msg;
            QString msgToSend = wssmsg;
            ldbg << "Sto inviando WSSMSG " << endl;

            if(wssmsg.getType() == WSSMessage::TYPE_ROBOT_DATA) {

                commenceDataExchange(wssmsg);
            } else {
                qint64 sentData = socket->write(msgToSend.toLatin1());
                ldbg << msgToSend << endl;
                socket->flush();

                if(sentData == -1) {
                    ldbg << "Errore 0" << endl;
                    emit sigError("Error sending data to WSS");
                }
                else{
                    ldbg << "Success!!" << endl;
                }
            }
        } else {
            ldbg << "Errore" << endl;
            emit sigError("WSS connection error");
        }
    }
    //m.unlock();
}

void WSSController::commenceDataExchange(const WSSMessage &msg)
{
    const QString &dest = msg.getPeer();
    const QByteArray &data = *msg.getRobotData();
    QString nextHop = router->getNextHopDestination(dest);
    if(nextHop == QString(RP_UNDEFINED) && msg.mustBeDelivered()){
        ldbg << "no next hop"<<endl;
        if(queuedData.contains(dest)){
            qDebug("DROP");
        }
        queuedData[dest].append(data);
    }
    else{
        ldbg << "else" << endl;
        if(connectionCache.contains(nextHop)) {
            QTcpSocket *sock = connectionCache[nextHop];
            int
                    done = sendData(nextHop, data);
            if(sock->state() == sock->ConnectedState) {
                if(done == -1)
                {
                    if(msg.mustBeDelivered()){
                        if(queuedData.contains(dest)){
                            qDebug("DROP1");
                        }
                        ldbg << "MSG Delevered compulsory0" << endl;                        
                        queuedData[dest].append(data);
                    }
                    else
                        ldbg << "Message couldn't be sent: operation failed" << endl;
                }
                ldbg << "msg sent!"<<endl;
            } else {
                if(msg.mustBeDelivered()){
                    ldbg << "MSG Delevered compulsory1" << endl;
                    if(queuedData.contains(dest)){
                        qDebug("DROP2");
                    }
                    queuedData[dest].append(data);
                }
                else
                    ldbg << "Message couldn't be sent: socket not connected to " << dest << endl;
            }
        } else {
            if(msg.mustBeDelivered()){
                if(queuedData.contains(dest)){
                    qDebug("DROP3");
                }
                ldbg << "MSG Delevered compulsory2" << endl;
                queuedData[dest].append(data);
            }
            else
                ldbg << "Message couldn't be sent: connection not initialized" << endl;  //Commented by Sajjad to reduce printing in logFile
        }
    }
}

void WSSController::initDataExchange(const QString &dest)
{

    if(connectionCache.contains(dest)&&connectionCache[dest]->state()==QAbstractSocket::ConnectedState){
        qDebug() << "La connessione esiste gi�!!!";
        return;
    }
    if(nRobot<=robotIndexFromName(dest)){
        qDebug() << "Lato sbagliato!!!";
        return;
    }
    connectionErrorTimer->stop();
    emit sigInitializeConnection(dest);


    if(!connectionCache.contains(dest)){

    QTcpSocket *sock = new QTcpSocket();
    //qDebug("INITDATA:");
    socketCaching(dest, sock);
    qDebug() << "Trying to connect to " << dest.toAscii().data();

    WSSMessage dnsRequest;
    dnsRequest.setType("DNS");
    dnsRequest["Robot"] = dest;
    //ldbg << "Sto inviando un messagfgio di tipo "<< dnsRequest.getType()+ " al robot "<< dnsRequest["Robot"] << " sulla porta " << dnsRequest["Port"]<< endl;
    sendMessage(dnsRequest);
    connectionErrorTimer->singleShot(TIME_TO_RECONNECT,this, SLOT(checkForConnetionErrors()));
    }
}

void WSSController::socketCaching(const QString &dest, QTcpSocket *sock)
{
    connectionCache[dest] = sock;

    connect(sock, SIGNAL(disconnected()), mapperDisconnected, SLOT(map()));
    connect(sock, SIGNAL(readyRead()), mapperData, SLOT(map()));

    mapperDisconnected->setMapping(sock, dest);
    mapperData->setMapping(sock, dest);

}

int WSSController::sendData(const QString &dest, const QByteArray &data)
{
    QTcpSocket *sock;
    if(connectionCache.contains(dest))
    {
        sock = connectionCache[dest];
        qint64 sentData = sock->write(data);
        sock->flush();

        if(sentData == -1) {
            emit sigError(QString("Error sending data to %1").arg(dest));
            return -1;
        }
        return 0;
    }
    return -1;
}

void WSSController::invokeReadData()
{
    while(socket->bytesAvailable() > 0) {
        if(socket->peek(2048).contains('\n')) {

            QString data = QString(socket->readLine());
            WSSMessage msg(data);
            ldbg << "Ho ricevuto " << msg.getPeer().toAscii().data() << " " << msg.getRobotData() << endl;
            //ldbg << "Ho ricevutio " << msg << " di tipo "<< msg.getType()+ " dal robot "<< msg["Robot"] << " dalla porta " << msg["Port"]<< endl;
            if (msg.getType() == "DNSREPLY"){
                quint16 robotPort = msg["Port"].toInt();
                QString robotName = msg["Robot"];
//                //ldbg << "arrived DNSREPLY message from " << robotName << " with port " << robotPort << endl;
                if(connectionCache.contains(robotName)) {
                    //ldbg << "Establishing connection with " << robotName << endl;
                    connectionCache[robotName]->connectToHost(
                                socket->peerName(), robotPort);
                }
            } else if(msg.getType() == "REVERSEDNSREPLY") {
                quint16 robotPort = msg["Port"].toInt();
                QString robotName = msg["Robot"];
                if(pendingReverseDNS.contains(robotPort)) {
                    qDebug("INVOKE READ CACHE:");
                    if(connectionCache.contains(robotName)&&connectionCache[robotName]->state()==QAbstractSocket::ConnectedState){
                        qDebug("Server connessione gi� presente!");
                    }
                    else{
                        socketCaching(robotName, pendingReverseDNS[robotPort]);
                        pendingReverseDNS.remove(robotPort);
                        robotData(robotName);
                    }
                }else{
                    qDebug("Error");
                }
            } else if(msg.getType() == "SS") {
                QString peer = msg["Robot"];
                QString strength = msg["Strength"];
                //qDebug()<<"Robot: " << peer.toAscii().data()<<" Strength: "<<strength.toAscii().data();
                if(strength != QString("INF") && strength != QString("NaN")){
                    double str = strength.toDouble();
                    if(peer==router->getNextHopDestination(robotNameFromIndex(BASE_STATION_ID))){
                        emit sigUpdateSignalStrength(QString::number(str));
                    }
                    /*if(str<-90){
                        qDebug("FC1");
                        forceClose(peer);
                    }*/
                    router->updateDirectConnection(peer, str);
                }else{
                    //qDebug("FC2");
                    //forceClose(peer);
                    router->updateDirectConnection(peer, -200);
                }
                emit sigMessage(msg);
            } else{

                emit sigMessage(msg);
            }
        } else {
            break;
        }
    }
}

/* This should be in WirelessDriver, but that implies changing too much stuff, so fuck that shit */
void WSSController::baseStationHandshake()
{
#ifdef __DEBUG__
    BuddyMessage handshake(robotNameFromIndex(nRobot),
                           robotNameFromIndex(BASE_STATION_ID), BuddyMessage::Empty);
    QByteArray data;
    QDataStream stream(&data, QIODevice::WriteOnly);
    stream << handshake;
    WSSMessage msg;
    msg.setRobotData(robotNameFromIndex(BASE_STATION_ID), &data);
    sendMessage(msg);
#endif
}

void WSSController::wssConnected()
{
    WSSMessage msg;
    msg.setType("INIT");
    msg["Robot"] = robotNameFromIndex(nRobot);
    msg["Port"] = QString::number(50001 + nRobot);
    qDebug("A");
    server->listen(QHostAddress::Any, 50001 + nRobot);
    qDebug("B");
    //ldbg << "Sto inviando " << msg << " di tipo "<< msg.getType()+ " al robot "<< msg["Robot"] << " sulla porta " << msg["Port"]<< endl;
    sendMessage(msg);
#ifdef __DEBUG__
    QTimer::singleShot(100, this, SLOT(baseStationHandshake()));
#endif
}

void WSSController::checkForConnectedPeers()
{
    int nrobots = Config::robotCount;
    for(int robotIndex = 0; robotIndex < nrobots; robotIndex++){
        if(robotIndex != nRobot)
        {
            WSSMessage msg;
            msg.setType("GETSS");
            msg["Robot"] = robotNameFromIndex(robotIndex);
             sendMessage(msg);
        }
    }
    if(nRobot != BASE_STATION_ID)
    {
        WSSMessage msg;
        msg.setType("GETSS");
        msg["Robot"] = robotNameFromIndex(BASE_STATION_ID);
         if(msg.contains("Robot")&&msg.contains("Port"))
        ldbg << "Sto inviando " << msg << " di tipo "<< msg.getType()+ " al robot "<< msg["Robot"] << " sulla porta " << msg["Port"]<< endl;
        sendMessage(msg);
    }
}

void WSSController::sendDistanceVector(QHash<QString,int> dv)
{

    DistanceVectorMessage dvMessage;
    BuddyMessage buddy;
    WirelessMessage msg;
    QHash<QString,int> customizedDv;
    foreach(QString dest, connectionCache.keys())
    {
        customizedDv=dv;
        foreach(QString target,dv.keys()){
            if(router->getNextHopDestination(target)==dest){
                customizedDv.remove(target);
            }
        }
        dvMessage = DistanceVectorMessage(robotNameFromIndex(nRobot),customizedDv);
        buddy= BuddyMessage(robotNameFromIndex(nRobot), dest, &dvMessage);
        msg = WirelessMessage(&buddy);
        ldbg << nRobot << dest.toAscii().data();
        emit sigSendDistanceVector(msg);
    }
}

void WSSController::onNewRoutingConnection(QString peer)
{
    //reliable handling of messages
    QString nextHop;
    if(queuedData.contains(peer)){
        nextHop = router->getNextHopDestination(peer);
        if(nextHop != RP_UNDEFINED){
            if(sendData(nextHop,queuedData[peer]) != -1)
            {
                queuedData.remove(peer);
                ldbg << "sent queued data to " << peer << " by mean of " << nextHop << endl;
            }
        }
    }
}

void WSSController::serverConnection()
{
    QTcpSocket *sock = server->nextPendingConnection();
    qDebug("CACHING");
    quint16 port = sock->peerPort();
    pendingReverseDNS[port] = sock;

    WSSMessage msg;
    msg.setType("REVERSEDNS");
    msg["Port"] = QString::number(port);
    //ldbg << "Sto inviando " << msg << " di tipo "<< msg.getType()+ " al robot "<< msg["Robot"] << " sulla porta " << msg["Port"]<< endl;
    sendMessage(msg);
}

void WSSController::robotDisconnected(const QString &name)
{
    qDebug() << "Ricevo disconnessione: " << name.toAscii().data();
    if(connectionCache.contains(name)) {
        mapperConnected->removeMappings(connectionCache[name]);
        mapperDisconnected->removeMappings(connectionCache[name]);
        mapperData->removeMappings(connectionCache[name]);
        connectionCache[name]->deleteLater();
        connectionCache.remove(name);
    }
    checkForConnectedPeers();
}

void WSSController::robotData(const QString &name)
{
    if(connectionCache.contains(name)) {
       QTcpSocket *sock = connectionCache[name];
       if(sock->bytesAvailable() > 0) {
           QByteArray data = sock->readAll();
           WSSMessage mex;
           mex.setRobotData(name, &data);
           emit sigMessage(mex);
       }
   }
}

void WSSController::debugCore()
{
    ldbg << "Routing: Routing table: " << " | Dest | nextHop" << endl << "--- | --- | ---" << endl ;
    for(int i = 0; i< Config::robotCount; i++){
        ldbg << "Routing:  | " << robotNameFromIndex(i) << " | " << router->getNextHopDestination(robotNameFromIndex(i)) << endl;
    }
    if (router->getNextHopDestination("Robot_666") == QString("und"))
        emit sigRobotNotReachable();
       ldbg << "Routing: | " << robotNameFromIndex(BASE_STATION_ID) << " | " <<
               router->getNextHopDestination(robotNameFromIndex(BASE_STATION_ID)) << endl;
    ldbg << "Routing: Actual connection cache: | " << endl;
    foreach(QString conn, connectionCache.keys()){
        ldbg << "Routing: | " << conn << " | ." << connectionCache[conn]->ConnectedState << endl;
    }
    ldbg << "Routing: | Messages actually queued for reliable handling: | ." << endl;
    foreach(QString dest, queuedData.keys()){
        ldbg << "Routing: " << dest << ": " << queuedData[dest].size() << " Bytes" << endl;
    }
//    ldbg << "Routing: Routing table: " << "Dest/t/tnextHop" << endl;
//    for(int i = 0; i< Config::robotCount; i++){
//        ldbg << "Routing: " << robotNameFromIndex(i) << ": " << router->getNextHopDestination(robotNameFromIndex(i)) << endl;
//    }
//       ldbg << "Routing: " << robotNameFromIndex(BASE_STATION_ID) << ": " <<
//               router->getNextHopDestination(robotNameFromIndex(BASE_STATION_ID)) << endl;
//    ldbg << "Routing: Actual connection cache: " << endl;
//    foreach(QString conn, connectionCache.keys()){
//        ldbg << "Routing: " << conn << ": " << connectionCache[conn]->ConnectedState << endl;
//    }
//    ldbg << "Routing: Messages actually queued for reliable handling: " << endl;
//    foreach(QString dest, queuedData.keys()){
//        ldbg << "Routing: " << dest << ": " << queuedData[dest].size() << " Bytes" << endl;
//    }

}

void WSSController::checkForConnetionErrors()
{
    QTcpSocket *sock;
    bool error = false;
    foreach(QString peer, connectionCache.keys()){
        sock = connectionCache[peer];
        if(sock->state() != sock->ConnectedState)
        {
            error = true;
            qDebug() << "Error on " << peer.toAscii().data();
            connectionCache.remove(peer);
            delete sock;
            router->updateDirectConnection(peer, -1000);
        }
        else{
            qDebug() << "Good on " << peer.toAscii().data();
        }
    }
    if(error)
        checkForConnectedPeers();
}

void WSSController::forceClose(QString robot){
    if(connectionCache.contains(robot)&&robotIndexFromName(robot)<nRobot){
        qDebug("0Force close!!");
        QTcpSocket* sock=connectionCache[robot];
        sock->disconnectFromHost();
        connectionCache.remove(robot);
        //checkForConnetionErrors();
    }
}

}

