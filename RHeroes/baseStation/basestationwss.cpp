#include "basestationwss.h"

#include <typeinfo>
#include <QTimer>
#include <QWaitCondition>


#include "shared/utilities.h"
#include "data/victimdelectionmessage.h"
#include "data/victimconfirmationmessage.h"
#include "data/robotvictimcouplingmessage.h"
#include "shared/config.h"
#include "data/victimdetectionconfmessage.h"
#include "data/errornotificationmessage.h"



using namespace Data;
using namespace Connection;
using namespace Middleware;
namespace BaseStation{


BaseStationWSS::BaseStationWSS(QObject *parent, QString bsLocation, int maxNRobots) :
    QObject(parent), usarController(new USARController()), wssController(new WSSController(BASE_STATION_ID)),
    wirelessDriver(new WirelessDriver(0, BASE_STATION_ID)), spawnedProcess(new QHash<uint, QProcess*>()),
    bsLocation(bsLocation), reachableRobots(new QList<uint>()), unreachableRobots(new QList<uint>()),
    maxRobots(maxNRobots)
{
    //Timerfor reachability checking
    reacheabilityTimer = new QTimer(this);

    //Reachability timer --> Basestation WSS (Reachability check
    connect(reacheabilityTimer, SIGNAL(timeout()), this, SLOT(reachabilityCheck()));

    reacheabilityTimer->start(BS_WSS_INACTIVITY_TIMEOUT);

    //USAR Controller --> Basestation WSS (Message, connection, error, disconnected)
    connect(usarController, SIGNAL(sigMessage(Data::Message)),
            this, SLOT(onMessage(Data::Message)));
    connect(usarController, SIGNAL(sigConnected()),
            this, SLOT(onConnected()));
    connect(usarController, SIGNAL(sigError(QString)),
            this, SLOT(onError(QString)));
    connect(usarController, SIGNAL(sigDisconnected()),
            this, SLOT(onDisconnected()));

    //WSS --> Basestation WSS
    connect(wssController, SIGNAL(sigError(QString)),
            this, SLOT(onError(QString)));
    /* WSS --> WirelessDriver */
    connect(wssController, SIGNAL(sigMessage(Data::Message)), wirelessDriver, SLOT(onMessageReceived(Data::Message)));
    connect(wssController, SIGNAL(sigSendDistanceVector(Data::Message)),
            wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    connect(wssController, SIGNAL(sigInitializeConnection(QString)), wirelessDriver, SLOT(flushBuffer(QString)),Qt::DirectConnection);
    /* WirelessDriver --> WSS */
    connect(wirelessDriver, SIGNAL(sigDriveMessageSend(Data::Message)), wssController, SLOT(sendMessage(Data::Message)));
    connect(wirelessDriver, SIGNAL(sigDistanceVector(QString,QHash<QString,int>)),
            wssController, SLOT(onDistanceVector(QString,QHash<QString,int>)));
    /* wirelessDriver --> BaseStation*/
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onMessage(Data::Message)));

}

BaseStationWSS::~BaseStationWSS()
{
    delete wssController;
    delete usarController;
    delete wirelessDriver;
    QProcess *proc;
    foreach(uint robot, spawnedProcess->keys())
    {
        proc = spawnedProcess->value(robot,0);
        spawnedProcess->remove(robot);
        if(proc!=0)
        {
            proc->terminate();
            delete proc;
        }
    }
    delete spawnedProcess;
}

/*-----------------------------------------------------------------------------------------------------------
                            Methods used to communicate with single robots
-----------------------------------------------------------------------------------------------------------*/

void BaseStationWSS::onWaypointCommand(Data::Pose waypoint, bool notifyWhenFinished, uint waitTime, uint robotID)
{
    WaypointCommand com = WaypointCommand(waypoint, notifyWhenFinished, waitTime);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(robotID),
                                      BuddyMessage::WaypointMessage, &com);
    WirelessMessage msg = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(msg);
    ldbg << "sent waypoint command to pose: " << waypoint.getX() << "," << waypoint.getY() << " to robot " << robotID << endl;
}

void BaseStationWSS::onHighLevelCommand(double x, double y, bool notifyWhenFinished, uint waitTime, uint robotID, bool isDirection)
{
    HighLevelCommand com = HighLevelCommand(x, y, notifyWhenFinished, waitTime,isDirection);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(robotID),
                                      BuddyMessage::HighCommand, &com);
    WirelessMessage msg = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(msg);
    ldbg << "sent high level command with: " << x << "," << y << " to robot " << robotID << endl;
}

void BaseStationWSS::onSendTeleoperationCommand(double sLeft, double sRight, uint RobotID)
{
    WheelMessage wheels = WheelMessage(sLeft, sRight);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID),
                                      robotNameFromIndex(RobotID), &wheels);
    WirelessMessage msg = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(msg);
    ldbg << "sent teleoperation message: " << sLeft << "," << sRight << endl;
}

void BaseStationWSS::onSendAirTeleoperationCommand(double altitude, double linear, double lateral,
                                               double rotational, uint robotID)
{
    AirDriveMessage air = AirDriveMessage(altitude, linear, lateral, rotational);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID),
                                      robotNameFromIndex(robotID), &air);
    WirelessMessage msg = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(msg);
    ldbg << "Sent air-teleoperation message: " << altitude << "," << linear << "," << lateral
         << "," << rotational << " to robot: " << robotID << endl;
}

void BaseStationWSS::onFramerateSetting(uint robot, bool highFrameRate)
{
    InfoMessage msg;
    if(highFrameRate)
        msg = InfoMessage(FRAMERATE_SETTING_COMMAND, FRAMERATE_HIGH);
    else
        msg = InfoMessage(FRAMERATE_SETTING_COMMAND, FRAMERATE_LOW);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(robot),
                                      BuddyMessage::InformationTransfer, &msg);
    WirelessMessage mex = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(mex);
    ldbg << "mandato set franerate a " << robot << endl;
}

void BaseStationWSS::onSendInfoMessage(uint robotID)
{
    InfoMessage msg = InfoMessage(INFORMATION_REQUEST);
    BuddyMessage buddy = BuddyMessage(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(robotID),
                                      BuddyMessage::InformationTransfer, &msg);
    WirelessMessage mex = WirelessMessage(&buddy);
    wirelessDriver->onDriverMessage(mex);
}

void BaseStationWSS::onVictimConfirmed(Pose position, uint victimID, uint robotID, bool isNew)
{
    VictimConfirmationMessage mex(position, victimID, isNew);
    int dest;
    bool found = false;
    if(robotID == BASE_STATION_ID)
    {
        for(int i = 0; i < maxRobots && !found; i++)
        {
            if(!unreachableRobots->contains(i))
            {
                dest = i;
                found = true;
            }
        }
    }
    else
    {
        found = true;
        dest = robotID;
    }
    if(found)
    {
        BuddyMessage buddy(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(dest), &mex);
        WirelessMessage message(&buddy, WirelessMessage::MessageExchange);
        wirelessDriver->onDriverMessage(message);
        ldbg << "victim " << victimID << " confirmed" << " and notified to the robot " <<
                robotNameFromIndex(dest) << endl;
    }
#warning: gestione del caso in cui la BS non Ã¨ connessa ad alcun robot!!! (TODO, bassa prioritÃ )
}

void BaseStationWSS::onVictimDeleted(uint victimID, uint robotID)
{
    Q_UNUSED(robotID);
    VictimDelectionMessage msg(victimID);
    for(int robot = 0; robot< Config::robotCount; robot++)
    {
        BuddyMessage buddy(robotNameFromIndex(BASE_STATION_ID),
                                               robotNameFromIndex(robot), &msg);
        WirelessMessage mex = WirelessMessage(&buddy, WirelessMessage::MessageExchange);
        wirelessDriver->onDriverMessage(mex);
        ldbg << "victim " << victimID << " deletion message sent to " << robot << endl;
    }
}

void BaseStationWSS::onVictimMoved(uint id, const Pose &position)
{
    ldbg << "victim " << id << " moved to: " << position.x() << "," << position.y() << endl;
    //WARNING: we suppose robot ids incremental and with no holes!
    for(int robot = 0; robot< Config::robotCount; robot++)
    {
        ldbg << "tell it to robot_" << robot << endl;
        onVictimConfirmed(position, id, robot, false);
    }
}

void BaseStationWSS::onVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                                    double hMax, double sMax, double vMax)
{
    VictimDetectionConfMessage msg = VictimDetectionConfMessage(hMin, sMin, vMin, hMax, sMax, vMax);
    for(int robotID = 0; robotID < maxRobots; robotID++)
    {
        BuddyMessage buddy = BuddyMessage(robotNameFromIndex(robotID), &msg);
        WirelessMessage mex = WirelessMessage(&buddy);
        wirelessDriver->onDriverMessage(mex);
        ldbg << "mandato messaggio di victim detection configuration a " << robotID
             << "con h tra " << hMin << " e " << hMax << endl;
    }
}

void BaseStationWSS::onRobotMessage(const Message &message, uint robotId)
{
    if(typeid(message) == typeid(WheelMessage)){
        const WheelMessage &msg = (const WheelMessage &)message;
        WheelMessage wheels(msg.getLeftWheelSpeed(), msg.getRightWheelSpeed());
        BuddyMessage buddy1(robotNameFromIndex(666), robotNameFromIndex(robotId), &wheels);
        WirelessMessage msg1(&buddy1);
        wirelessDriver->onDriverMessage(msg1);
    }
}

void BaseStationWSS::onChangeModuleStatusBW(const QString &module, bool enable)
{
    //TODO
    ldbg << "Module activation message: ";
    if(enable)
        ldbg << "enable ";
    else
        ldbg << "disable ";
    ldbg << module << endl;
    ModuleActivationMessage msg(module, enable);
    //forall robots... reliable message sending!!! (TODO)
    for(int i = 0; i< maxRobots; i++)
    {
        if(enable)
            ldbg << "Module " << module << " enabled" << endl;
        else
            ldbg << "Module " << module << " disabled" << endl;
        BuddyMessage buddy(robotNameFromIndex(BASE_STATION_ID),
                                               robotNameFromIndex(i), BuddyMessage::ModuleActivation, &msg);
        WirelessMessage mex = WirelessMessage(&buddy, WirelessMessage::MessageExchange);
        wirelessDriver->onDriverMessage(mex);
    }
}

void BaseStationWSS::onChangeModuleStatusSpecificBW(const QString &module, bool enable, uint robotID)
{
    if(enable)
        ldbg << "Module " << module << " enabled" << endl;
    else
        ldbg << "Module " << module << " disabled" << endl;
    ModuleActivationMessage msg(module, enable);
    BuddyMessage buddy(robotNameFromIndex(BASE_STATION_ID),
                                           robotNameFromIndex(robotID), BuddyMessage::ModuleActivation, &msg);
    WirelessMessage mex = WirelessMessage(&buddy, WirelessMessage::MessageExchange);
    wirelessDriver->onDriverMessage(mex);
}

void BaseStationWSS::onMapNotWorking(uint robot)
{
    ldbg << "sending map KO message to " << robot << endl;
    ErrorNotificationMessage msg(ErrorNotificationMessage::MapKO);
    BuddyMessage buddy(robotNameFromIndex(BASE_STATION_ID), robotNameFromIndex(robot),
                       BuddyMessage::ErrorNotification, &msg);
    WirelessMessage mex(&buddy, WirelessMessage::MessageExchange);
    wirelessDriver->onDriverMessage(mex);
}

void BaseStationWSS::reachabilityCheck()
{
    for(int i = 0; i < maxRobots; i++){
        if(! reachableRobots->contains(i) && !unreachableRobots->contains(i))
        {
            unreachableRobots->append(i);
            emit sigRobotUnreachableBW(i);
        }
    }
    reachableRobots->clear();
}

void BaseStationWSS::killProcess(const QString &location, const QString &rotation,
                                 const QString &bsLocation, uint nActiveRobot, uint maxRobots, bool aerial, bool kenaf)
{
    QProcess *p = this->spawnedProcess->value(nActiveRobot,0);
    if(p != 0)
    {
        spawnedProcess->remove(nActiveRobot);
        p->close();
        //delete p;
        this->respawnLocation = location;
        this->respawnRotation = rotation;
        this->bsLocation = bsLocation;
        this->robotToRespawn = nActiveRobot;
        this->maxRobots = maxRobots;
        this->respawnAerial = aerial;
        this->respawnKenaf = kenaf;
        QTimer t;
        t.singleShot(20000, this, SLOT(respawnProcess()));
    }
}

void BaseStationWSS::respawnProcess()
{
    this->onSpawnRobotBW(respawnLocation, respawnRotation, bsLocation, robotToRespawn, maxRobots, respawnAerial, respawnKenaf);
}

void BaseStationWSS::onMessage(const Data::Message &message)
{
//-------------------------------------------------Usar message-----------------------------------//
    if(typeid(message) == typeid(USARMessage)){
        const USARMessage &msg = (const USARMessage &)message;
        if(msg.contains("StartPoses")) {
            foreach(QString chiave, msg.keys()) {
                if(chiave != "StartPoses") {
                    QString joined = chiave + " " + msg[chiave];
                    QStringList params = joined.split(" ");
                    QString temp;
                    for(int i = 0; i < params.size(); i+=3) {
                        temp = params[i]+ "," + params[i+1] + "," + params[i+2];
                        emit sigInitialLocation(temp);
                    }
                    break;
                }
            }
        }
    }
//---------------------------------------------------Wireless messages----------------------------//
    if(typeid(message) == typeid(WirelessMessage)){
        const WirelessMessage &msg = (const WirelessMessage &)message;
        if(msg.getCommand() == WirelessMessage::MessageExchange){
            const BuddyMessage *buddy = msg.getBuddyMessage();
            QString source = buddy->getSource();
            uint id;
            if(source.contains("robot", Qt::CaseInsensitive))
                id = robotIndexFromName(source);
            else {
                ldbg<<"Sto ritornando. source vale: " << source<<endl;
                return; }
            if(!reachableRobots->contains(id))
                reachableRobots->append(id);
            if(unreachableRobots->contains(id)){
                unreachableRobots->removeAll(id);
                //unreachableRobots->removeAt(unreachableRobots->indexOf(id));
                emit sigRobotReachableAgain(id);
            }

//            ldbg << "received message" << buddy->getContent() << endl;
            if(buddy->getContent() == BuddyMessage::CameraFrame){              //Camera message
                const CameraData *camData = buddy->getCameraData();
                emit sigCameraDataReceived(id, camData->getFrame());
            }
            else if(buddy->getContent() == BuddyMessage::VictimInformation){        //Victim Detected
                const VictimMessage *victimMex = buddy->getVictimMessage();
                QImage img = victimMex->getVictimImage();
                double confidence = victimMex->getConfidence();
                Pose position = victimMex->getPosition();
                emit sigVictimMessage(id, img, confidence, position);
            }
            else if(buddy->getContent() == BuddyMessage::MapInformation){           //Map Message
                const SLAM::Map *mapData = buddy->get<SLAM::Map>();
                emit sigNewMapDataReceived(*mapData, id);
            }
            else if(buddy->getContent() == BuddyMessage::RobotVictimCoupling){          //Victim allocation
                const RobotVictimCouplingMessage *data = buddy->get<RobotVictimCouplingMessage>();
                ldbg << "Victim assignation message incoming" << endl;
                emit sigVictimAssigned(data->getVictimId(), id, data->getTimeToReach());
            }
            else if(buddy->getContent() == BuddyMessage::InformationTransfer)        //Info message
            {
                const InfoMessage *mex = buddy->getInfoMessage();
                if(mex->getInfoName() == MOVEMENT_END)
                {
                    ldbg << "received waypoint reached message from " << source << endl;
                    emit sigWaypointReached(id);
                }
                emit sigInfoMessageReceived(*mex, id);
            }
            else if(buddy->getContent() == BuddyMessage::ErrorNotification)             //Error message
            {
                ldbg << "Arrived fault message!!!" << endl;
                const ErrorNotificationMessage *mex = buddy->get<ErrorNotificationMessage>();
                QString message = "lol, error somewhere!";
                switch(mex->getInvolvedModule()){
                case ErrorNotificationMessage::PathPlanner: message = "Path not found!"; break;
                case ErrorNotificationMessage::Navigation: message = "Robot blocked by something!"; break;
                case ErrorNotificationMessage::Exploration: message = "Exploration algorithm fault"; break;
                case ErrorNotificationMessage::VictimDetection: message = "Victim detection module error"; break;
                case ErrorNotificationMessage::Unknown: message = "Unknown error!"; break;
                }
                emit sigFaultDetected(message, id);
            }
            else if(buddy->getContent() == BuddyMessage::DestinationNotificationMessage){
                const DestinationMessage *mex = buddy->get<DestinationMessage>();
                uint id = mex->getRobotId();
                SLAM::Geometry::Point p = mex->getPoint();
                ldbg << "arrived destination message from " << id <<
                        ": going to (" << p.x() << "," << p.y() << ")" << endl;
                emit sigNewDestinationReceived(id,Pose(p.x(), p.y(),0));
            }
            else if(buddy->getContent() == BuddyMessage::Empty){
                ldbg << "incoming connection from " << source << endl;
                sigIncomingRobotConnectionBW(id);
            }

        } else if(msg.getCommand() == WirelessMessage::NeighbourQuery){      //Handshake message

        }
    }
}



/*--------------------------------------------------------------------------------------------------
                        Slots used to manage the connection establishment
--------------------------------------------------------------------------------------------------*/

void BaseStationWSS::onWSSConnect(const QString &wssAddress, const quint16 wssPort)
{
    this->wssAddress = wssAddress;
    this->wssPort = wssPort;
}

void BaseStationWSS::onUPISConnect(const QString &upisAddress, const quint16 upisPort)
{
    this->upisAddress = upisAddress;
    this->upisPort = upisPort;
}

void BaseStationWSS::onUSARConnect(const QString &usarAddress, const quint16 usarPort)
{
    this->usarAddress = usarAddress;
    this->usarPort = usarPort;
    usarController->connectToHost(usarAddress,usarPort);
}

void BaseStationWSS::onUSARDisconnect()
{
    usarController->disconnectFromHost();
}

void BaseStationWSS::onWSSDisconnect()
{
    wssController->disconnectFromHost();
}

void BaseStationWSS::onConnected()
{
    QTimer::singleShot(100, this, SLOT(onConnectedAndWaited()));
}

void BaseStationWSS::onConnectedAndWaited()
{
    QString initString = "GETSTARTPOSES";
    USARMessage message(initString);
    usarController->sendMessage(message);
    emit sigConnectedBW();
}

void BaseStationWSS::onError(QString error)
{
    emit sigErrorBW(error);
}

void BaseStationWSS::onSendRawCommand(Data::Message mex)
{
    this->usarController->sendMessage(mex);
}

void BaseStationWSS::onSpawnBS(QString location)
{
    bsLocation = location;
    QString loc = SPAWN_STATIC_R_USAR;
    loc += "{Location " + bsLocation + "}";
    ldbg<<loc<<endl;
    usarController->sendMessage(USARMessage(loc));
    wssController->connectToHost(wssAddress, wssPort);
    //QTimer::singleShot(5000, this, SLOT(dummySlot()));


}

void BaseStationWSS::onDisconnected()
{
    emit sigDisconnectedBW();
}

void BaseStationWSS::onSpawnRobotBW(const QString &location, const QString &rotation,
                                  const QString &bsLocation, uint nActiveRobot, uint maxRobots, bool aerial, bool isKenaf)
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
    arguments << "--rotation" << rotation;
    arguments << "--bs-location" << bsLocation;
    arguments << "--id" << QString::number(nActiveRobot);
    arguments << "--n-robots" << QString::number(maxRobots);
    if(aerial)
        arguments << "--type" << "air";
    else
    {
        if(isKenaf)
            arguments << "--type" << "kenaf";
        else
            arguments << "--type" << "p3at";
    }

    //Starting the process...
    /*connect(proc, SIGNAL(error(QProcess::ProcessError)), this, SLOT(onProcessError(QProcess::ProcessError)));
    connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(onProcessReadyRead()));
    connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(onProcessReadyRead()));*/
    proc->setStandardOutputFile("poaret_out.log", QIODevice::Append);
    proc->setStandardErrorFile("poaret_err.txt", QIODevice::Append);
    proc->startDetached(program, arguments);

    spawnedProcess->insert(nActiveRobot,proc);
}

}



