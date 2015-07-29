#include "basestation.h"
#include "data/usarmessage.h"
#include "data/wheelmessage.h"
#include "data/wirelessmessage.h"
#include "graphics/robotui.h"
#include "data/buddymessage.h"
#include "shared/utilities.h"
#include <QDebug>
#include <typeinfo>
#include "shared/constants.h"
#include "shared/config.h"

using namespace Connection;
using namespace Data;
using namespace Middleware;

namespace Logic{

BaseStation::BaseStation(QObject *parent) :
    QObject(parent), usarController(new USARController()), wssController(new WSSController(666)),
    wirelessDriver(new WirelessDriver(0, BASE_STATION_ID)), nActiveRobot(0), graphics(new BaseStationUi()), spawnedRobots(new QStack<RobotUi *>),
    spawnedProcess(new QStack<QProcess *>())
{

    connect(this, SIGNAL(signalInitialLocation(QString)), graphics, SLOT(onInitialLocation(QString)));

    /* graphics --> USARSim */
    connect(graphics, SIGNAL(signalUSARConnect(QString,quint16)), this, SLOT(onUSARConnect(QString,quint16)));
    connect(graphics, SIGNAL(signalUSARConnect(QString,quint16)), usarController, SLOT(connectToHost(QString,quint16)));
    connect(graphics, SIGNAL(signalDisconnect()), usarController, SLOT(disconnectFromHost()));
    connect(graphics, SIGNAL(signalMessage(Data::Message)), usarController, SLOT(sendMessage(Data::Message)));
    /* graphics --> UPIS */
    connect(graphics, SIGNAL(signalUPISConnect(QString,quint16)), this, SLOT(onUPISConnect(QString,quint16)));
    /* graphics --> WSS */
    //connect(graphics, SIGNAL(signalWSSConnect(QString, quint16)), wssController, SLOT(connectToHost(QString,quint16)));
    connect(graphics, SIGNAL(signalWSSConnect(QString, quint16)), this, SLOT(onWSSConnect(QString,quint16)));
    connect(graphics, SIGNAL(signalDisconnect()), wssController, SLOT(disconnectFromHost()));
    /* graphics --> this */
    connect(graphics, SIGNAL(signalSpawnRobot(QString)), this, SLOT(onSpawnRobot(QString)));
    //connect(graphics, SIGNAL(signalDisconnect()) usarController, SLOT(disconnectFromHost()));

    /* USARSim --> graphics */
    connect(usarController, SIGNAL(signalDisconnected()), graphics, SLOT(onDisconnected()));
    connect(usarController, SIGNAL(signalConnected()), graphics, SLOT(onConnected()));
    connect(usarController, SIGNAL(signalError(QString)), graphics, SLOT(appendOnConsole(QString)));
    connect(usarController, SIGNAL(signalMessage(Data::Message)), graphics, SLOT(appendOnConsole(Data::Message)));

    /* USARSim --> this */
    connect(usarController, SIGNAL(signalMessage(Data::Message)), this, SLOT(onMessage(Data::Message)));
    connect(usarController, SIGNAL(signalConnected()), this, SLOT(onConnected()));

    /* WSS --> graphics */
    connect(wssController, SIGNAL(signalError(QString)), graphics, SLOT(appendOnConsole(QString)));

    /* WSS --> WirelessDriver */
    connect(wssController, SIGNAL(signalMessage(Data::Message)), wirelessDriver, SLOT(onMessageReceived(Data::Message)));
    /* WirelessDriver --> WSS */
    connect(wirelessDriver, SIGNAL(lowLevelCommand(Data::Message)), wssController, SLOT(sendMessage(Data::Message)));

    /* wirelessDriver --> BaseStation*/
    connect(wirelessDriver, SIGNAL(sensorData(Data::Message)), this, SLOT(onMessage(Data::Message)));

    graphics->show();
}

BaseStation::~BaseStation()
{
    while(!spawnedRobots->isEmpty()){
        delete spawnedRobots->pop();
    }
    while(!spawnedProcess->isEmpty()){
        QProcess *proc = spawnedProcess->pop();
        proc->terminate();
        delete proc;
    }
    delete spawnedProcess;
    delete spawnedRobots;
    delete wirelessDriver;
    delete wssController;
    delete usarController;
    delete graphics;
}

void BaseStation::onUSARConnect(const QString &usarAddress, const quint16 usarPort)
{
    this->usarAddress = usarAddress;
    this->usarPort = usarPort;

}

void BaseStation::onUPISConnect(const QString &upisAddress, const quint16 upisPort)
{
    this->upisAddress = upisAddress;
    this->upisPort = upisPort;
}


void BaseStation::onWSSConnect(const QString &wssAddress, const quint16 wssPort) {
    this->wssAddress = wssAddress;
    this->wssPort = wssPort;
}

void BaseStation::onSpawnRobot(const QString &location)
{

    //process creation
    QProcess* proc = new QProcess();
    QString program = Config::poaretBinary;
    QStringList arguments;
    //arguments creation
    arguments << "--usar-port" << QString::number(usarPort);
    arguments << "--usar-addr" << usarAddress;
    arguments << "--upis-port" << QString::number(upisPort);
    arguments << "--upis-addr" << upisAddress;
    arguments << "--wss-port" << QString::number(wssPort);
    arguments << "--wss-addr" << wssAddress;
    arguments << "--location" << location;
    arguments << "--id" << QString::number(nActiveRobot);

    //Starting the process...
    connect(proc, SIGNAL(error(QProcess::ProcessError)), this, SLOT(onProcessError(QProcess::ProcessError)));
    connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(onProcessReadyRead()));
    connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(onProcessReadyRead()));
    proc->start(program, arguments);

    spawnedProcess->push(proc);

    //robotUi Creation
    RobotUi* rUi = new RobotUi(nActiveRobot);
    spawnedRobots->push(rUi);
    nActiveRobot++;
    rUi->show();

    //Connect the signals that comes from the RobotUI to the BaseStation.
    //In this way, the base station can build the message for the robot and send the
    //message to it throught the wss.
    connect(rUi, SIGNAL(emitMessage(Data::Message, uint)), this, SLOT(onRobotMessage(Data::Message, uint)));
}

void BaseStation::onConnected()
{
    QString initString = "GETSTARTPOSES";
    USARMessage message(initString);
    usarController->sendMessage(message);

    //I have to be allowed to mosify the starting location for the BaseStation 'robot'
    usarController->sendMessage(USARMessage(SPAWN_STATIC_R_USAR));
    wssController->connectToHost(wssAddress, wssPort);
}

void BaseStation::onMessage(const Message &message)
{
    if(typeid(message) == typeid(USARMessage)){
        const USARMessage &msg = (const USARMessage &)message;
        if(msg.contains("StartPoses")) {
            foreach(QString chiave, msg.keys()) {
                if(chiave != "StartPoses") {
                    QString joined = chiave + " " + msg[chiave];
                    QStringList params = joined.split(" ");
                    for(int i = 0; i < params.size(); i += 3) {
                        emit signalInitialLocation(params[i]);
                    }
                    break;
                }
            }
        }
    }
    if(typeid(message) == typeid(WirelessMessage)){
        //qDebug()<<"recv a msg from the wireless";
        const WirelessMessage &msg = (const WirelessMessage &)message;
        if(msg.getCommand() == WirelessMessage::MessageExchange){
            const BuddyMessage *buddy = msg.getBuddyMessage();
            if(buddy->getContent() == BuddyMessage::CameraFrame){
                const CameraData *camData = buddy->getCameraData();
                QString source = buddy->getSource();
                uint id = robotIndexFromName(source);

                RobotUi * rUi = spawnedRobots->at(id);
                rUi->onImage(camData->getFrame());
            }


        } else if(msg.getCommand() == WirelessMessage::NeighbourQuery){

        }
    }
}

void BaseStation::onRobotMessage(const Message &message, uint robotId)
{

    //Check the message type.
    qDebug()<<"Message received from RobotUi";
    if(typeid(message) == typeid(WheelMessage)){
        const WheelMessage &msg = (const WheelMessage &)message;
        qDebug()<<"Wheel Message with l_speed: "<<msg.getLeftWheelSpeed()<<" and r_speed: "<<msg.getRightWheelSpeed()<<endl;

        WheelMessage ruote(msg.getLeftWheelSpeed(), msg.getRightWheelSpeed());
        BuddyMessage buddy1(robotNameFromIndex(666), robotNameFromIndex(robotId), &ruote);
        qDebug() << "DESTINATION: "<< robotNameFromIndex(robotId);
        WirelessMessage msg1(&buddy1);
        qDebug()<<"try to send a message on driver" << endl;
        wirelessDriver->onDriverMessage(msg1);
        //Create a WirelessMessage to send it to the right robot
        //through the wssController
    }

}

void BaseStation::onProcessError(QProcess::ProcessError error){
    qDebug() << "Error recv: "<< error;
}

void BaseStation::onProcessReadyRead(){
    QProcess *proc = spawnedProcess->at(0);
    QByteArray qstdout = proc->readAllStandardOutput(),
            qstderr = proc->readAllStandardError();
    if(qstdout.size() > 0)
        qDebug() << qstdout;
    if(qstderr.size() > 0)
        qDebug() << qstderr;
}


}
