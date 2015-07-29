#include "abstractsocketcontroller.h"
#include "data/usarmessage.h"
#include <QHostAddress>

namespace Connection {

AbstractSocketController::AbstractSocketController(QObject *parent) :
    QObject(parent),port(0)
{
    socket = new QTcpSocket();
    connect(socket, SIGNAL(connected()), this, SLOT(catchConnected()));
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(catchError(QAbstractSocket::SocketError)));
    connect(socket, SIGNAL(disconnected()), this, SLOT(catchDisconnected()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(invokeReadData()));
}

AbstractSocketController::~AbstractSocketController()
{
    socket->disconnectFromHost();
    socket->close();
    delete socket;
}

void AbstractSocketController::connectToHost(
        const QString &address, quint16 port)
{
    addr=QHostAddress(address);
    this->port=port;
    socket->connectToHost(addr, port);
}

void AbstractSocketController::disconnectFromHost()
{
    if(socket->state() == socket->ConnectedState) {
        socket->disconnectFromHost();
    }
}

void AbstractSocketController::catchConnected()
{
    emit sigConnected();
}

void AbstractSocketController::catchError(QAbstractSocket::SocketError error)
{
    QString err;
    switch(error){
        default:
            err = "Connection error: ";
            err.append(socket->errorString());
        break;
    }

    emit sigError(err);
}

void AbstractSocketController::catchDisconnected()
{
    emit sigDisconnected();
}

void AbstractSocketController::reconnectToHost(){
    if(port!=0){
        socket->connectToHost(addr, port);
    }
}
}
