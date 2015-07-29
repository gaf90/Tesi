#include "upiscontroller.h"
#include <QHostAddress>
#include "data/usarmessage.h"
#include "shared/utilities.h"
#include "data/upismessage.h"

#define UPIS_MAX_CAMERAS_ON_A_LINE   4
#define UPIS_CAMERA_WIDTH            320
#define UPIS_CAMERA_HEIGHT           240

namespace Connection {

UPISController::UPISController(double fps, int nRobot, QObject *parent) :
    AbstractSocketController(parent), timer(this), fps(fps), reading(false),
    x((nRobot * UPIS_CAMERA_WIDTH) %
      (UPIS_CAMERA_WIDTH * UPIS_MAX_CAMERAS_ON_A_LINE)),
    y(nRobot / UPIS_MAX_CAMERAS_ON_A_LINE * UPIS_CAMERA_HEIGHT)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(requestFrame()));
}

UPISController::~UPISController()
{
}

void UPISController::setFPS(double fps)
{
    this->fps = fps;
    timer.start(1000. / fps);
}

void UPISController::connectToHost(
        const QString &address, quint16 port)
{
    timer.start(1000. / fps);
    AbstractSocketController::connectToHost(address, port);
}

void UPISController::disconnectFromHost()
{
    timer.stop();
    AbstractSocketController::disconnectFromHost();
}

void UPISController::invokeReadData()
{
    if(!reading) {
        if(socket->bytesAvailable() >= 5) {
            quint8 type;
            QByteArray binData = socket->read(5);
            QDataStream data(&binData, QIODevice::ReadOnly);
            data.setByteOrder(QDataStream::BigEndian);

            data >> type >> sizeLeft;

            jpeg = (type > 0);
            reading = true;

            invokeReadData();
        }
    } else {
        uint32_t nBytes = min<uint32_t>(sizeLeft, socket->bytesAvailable());
        sizeLeft -= nBytes;
        rawData.append(socket->read(nBytes));
        if(sizeLeft == 0) {
            reading = false;
            emit sigMessage(Data::UPISMessage(jpeg, rawData));
            rawData.clear();
        }
        if(socket->bytesAvailable() > 0) {
            invokeReadData();
        }
    }
}

void UPISController::requestFrame()
{
    if(socket->state() == socket->ConnectedState) {
        QByteArray binData;
        QDataStream data(&binData, QIODevice::WriteOnly);
        data.setByteOrder(QDataStream::BigEndian);
        data << 'U' << x << y << (uint32_t) UPIS_CAMERA_WIDTH <<
                (uint32_t) UPIS_CAMERA_HEIGHT;
        qint64 sentData = socket->write(binData);
        socket->flush();
        if(sentData == -1) {
            emit sigError("Error sending data to UPIS");
        }
    } else {
        emit sigError("UPIS connection error");
    }
}

}
