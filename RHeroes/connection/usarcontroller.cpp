#include "usarcontroller.h"
#include "data/usarmessage.h"
#include "shared/logger.h"
#include "shared/config.h"
#include <typeinfo>
#include <QTime>
#include <QFile>

#define DATASET_GENERATION

namespace Connection {

using namespace Data;

#ifdef DATASET_GENERATION
QFile *datasetFile;
#endif

USARController::USARController(QObject *parent) :
    AbstractSocketController(parent)
{
#ifdef DATASET_GENERATION
    datasetFile = new QFile(QString("poaret_dataset_%1.txt").arg(Config::robotID));
    datasetFile->open(QIODevice::WriteOnly);
#endif
}

USARController::~USARController()
{
}

void USARController::sendMessage(const Message &msg)
{
    if(typeid(msg) == typeid(const USARMessage &)) {
        if(socket->state() == socket->ConnectedState) {
            QString msgToSend = (const USARMessage &) msg;
            qint64 sentData = socket->write(msgToSend.toLatin1());
            socket->flush();

            if(sentData == -1) {
                emit sigError("Error sending data to USARSim");
            }
        } else {
            emit sigError("USARSim connection error");
        }
    }
}

void USARController::invokeReadData()
{
    int totalSize=0;
    while(socket->bytesAvailable() > 0) {
        if(socket->peek(10000).contains('\n')) {
            QByteArray data = socket->readLine();
            totalSize+=data.size();

#ifdef DATASET_GENERATION
            datasetFile->write(data);
            datasetFile->flush();
#endif
            emit sigMessage(USARMessage(QString(data)));
        } else {
            //qDebug("%s - %i",QTime::currentTime().toString().toAscii().data(),totalSize);
            break;
        }
    }
}

}
