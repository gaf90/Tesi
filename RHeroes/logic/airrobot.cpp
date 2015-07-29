#include "airrobot.h"

#ifndef TESTING_GROUNDTRUTH
#   include "middleware/sensor/odometrysensor.h"
#else
#   include "middleware/sensor/groundtruthsensor.h"
#endif
#include "middleware/sensor/camerasensor.h"
#include "middleware/sensor/statesensor.h"
#include "middleware/driver/wirelessdriver.h"
#include "data/usarmessage.h"
#include "data/wirelessmessage.h"
#include "shared/mutexqueue.h"
#include "shared/config.h"
#include "data/infomessage.h"
#include "data/buddymessage.h"
#include <typeinfo>

using namespace Connection;
using namespace Data;
using namespace Middleware;


AirRobot::AirRobot(const QString &location, const QString &rotation, const uint identifier, QObject *parent) :
    QObject(parent), robotController(new RobotController(identifier, location, rotation,false)),
    usarController(new USARController()), upisController(new UPISController(2, identifier)),
    wssController(new WSSController(identifier)), sensors(new QList<Sensor *>()), drivers(new QList<Driver *>()),
    initialLocation(location), initialRotation(rotation), identifier(identifier)
{
//    upisController->setFPS(6);

    //Add sensors and driver to the robot
#ifndef TESTING_GROUNDTRUTH
//    sensors->append(new OdometrySensor());
#else
    sensors->append(new GroundTruthSensor());
#endif
    CameraSensor *cameraSensor = new CameraSensor();
    sensors->append(cameraSensor);
    connect(cameraSensor, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onSensorData(Data::Message)));
    sensors->append(new StateSensor());

//    drivers->append(new AirControlDriver());
    airDriver = new AirControlDriver();
    connect(airDriver, SIGNAL(sigDriveMessageSend(Data::Message)), usarController, SLOT(sendMessage(Data::Message)));
    wirelessDriver = new WirelessDriver(0, identifier);
    //Handle sensors' connection with controller
    for(int i=0; i<sensors->size(); i++){
        connect(usarController, SIGNAL(sigMessage(Data::Message)), sensors->at(i), SLOT(onMessageReceived(Data::Message)));
        connect(upisController, SIGNAL(sigMessage(Data::Message)), sensors->at(i), SLOT(onMessageReceived(Data::Message)));
        connect(sensors->at(i), SIGNAL(sigSensorData(Data::Message)), robotController, SLOT(onSensorData(Data::Message)));

        // Send everything to the SLAM module too, Mladen likes that, Mladen doesn't want to modify
        // RobotController because he's really lazy (and also a bit dumbass)
    }

    //Handle drivers' connection with controller
    for(int i=0; i<drivers->size(); i++){
        connect(drivers->at(i), SIGNAL(sigDriveMessageSend(Data::Message)), usarController, SLOT(sendMessage(Data::Message)));
        connect(robotController, SIGNAL(sigDriverMessageRobot(Data::Message)), drivers->at(i), SLOT(onDriverMessage(Data::Message)));
    }

    //Handle connection with the controller
    // --> after Connect
    connect(usarController, SIGNAL(sigModuleConnected()), this, SLOT(onUSARSimConnected()));
    // --> after Disconnect
    connect(usarController, SIGNAL(sigModuleDisconnected()), this, SLOT(onUSARSimDisconnected()));

    connect(upisController, SIGNAL(sigModuleConnected()), this, SLOT(onUPISSimConnected()));
    // --> after Disconnect
    connect(upisController, SIGNAL(sigModuleDisconnected()), this, SLOT(onUPISDisconnected()));

    connect(upisController, SIGNAL(sigError(QString)), this, SLOT(onUPISError(QString)));

    /* WSS --> WirelessDriver */
    connect(wssController, SIGNAL(sigMessage(Data::Message)), wirelessDriver, SLOT(onMessageReceived(Data::Message)));
    connect(wssController, SIGNAL(sigSendDistanceVector(Data::Message)),
            wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    /* WirelessDriver --> WSS */
    connect(wirelessDriver, SIGNAL(sigDriveMessageSend(Data::Message)), wssController, SLOT(sendMessage(Data::Message)));
    connect(wirelessDriver, SIGNAL(signalDistanceVector(QString,QHash<QString,int>)),
            wssController, SLOT(onDistanceVector(QString,QHash<QString,int>)));

    /* wirelessDriver --> robot*/
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), robotController, SLOT(onSensorData(Data::Message)));
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onSensorData(Data::Message)));
    /* Robot --> WirelessDriver */
    connect(robotController, SIGNAL(sigWirelessMessageRCM(Data::Message)), wirelessDriver, SLOT(onDriverMessage(Data::Message)));
//    connect(this, SIGNAL(sigWaypointReceived(const Data::WaypointCommand*)), robotController, SLOT(handleWaypoint(const Data::WaypointCommand*)));
}

AirRobot::~AirRobot()
{
    for(int i=drivers->size()-1; i>=0; i--){
        delete drivers->at(i);
    }
    delete drivers;

    for(int i=sensors->size()-1; i>=0; i--){
        delete sensors->at(i);
    }
    delete sensors;
    delete wirelessDriver;
    delete wssController;
    delete upisController;
    delete usarController;
    delete robotController;
}

void AirRobot::spawnRobot()
{
    //connect the robot to USARSim
    ldbg << "Usar Address:Port " << Config::usarAddress<< ":"<<Config::usarPort<<endl;
    usarController->connectToHost(Config::usarAddress, Config::usarPort);

    //Connect to upis
    ldbg << "Upis Address:Port " << Config::upisAddress<< ":"<<Config::upisPort<<endl;
    upisController->connectToHost(Config::upisAddress, Config::upisPort);

    //Connect to WSS
    ldbg << "WSS Address:Port " << Config::wssAddress<< ":"<<Config::wssPort<<endl;
    wssController->connectToHost(Config::wssAddress, Config::wssPort);
}

void AirRobot::onPowerSignalDataGathered()
{       //Da modificare per ricavarci delle informazioni sulla connettività dell'airrobot!
    //    if(shouldSendData){
        QString data;
        QHash<uint, double> *powerTable = robotController->getActualState()->getSignalPowerData();
        bool first=true;
        foreach(uint key, powerTable->keys()){
            if(first)
               first = false;
            else
                data.append(QString(";"));
            data.append(QString::number(key)).append(QString("=")).append(QString::number(powerTable->value(key)));
         }
        ldbg << "Power data = "<<data<<endl;
        InfoMessage info(CONNECTION_INFORMATION, data);
        BuddyMessage buddy(robotNameFromIndex(identifier), robotNameFromIndex(BASE_STATION_ID), &info);
        WirelessMessage ws(&buddy);
        wirelessDriver->onDriverMessage(ws);
        powerTable->clear();
        delete powerTable;
    //    }
}

void AirRobot::onUSARSimConnected()                                     //TODO: messaggio come si deve!!!
{
    //Spawn the robot in the simulation
    USARMessage message;
    message.setType("INIT");
    message.insert("ClassName", "USARBot.AirRobot");
    message.insert("Name", robotNameFromIndex(identifier));
    message.insert("Location", initialLocation);
    message.insert("Rotation", initialRotation);
    usarController->sendMessage(message);
}

void AirRobot::onUSARSimDisconnected()
{
}

void AirRobot::onSensorData(const Data::Message &msg)
{
    if(typeid(msg) == typeid(const WirelessMessage &)){
        const WirelessMessage &message = (const WirelessMessage &)msg;
        if (message.getCommand() == WirelessMessage::MessageExchange){
            const BuddyMessage *buddy = message.getBuddyMessage();
            //Provvisorio, vedete voi dove metterlo
            if(buddy->getContent() == BuddyMessage::ModuleActivation){
                const ModuleActivationMessage* moduleActivation = buddy->get<ModuleActivationMessage>();
                QString moduleName = moduleActivation->getInvolvedModule();
                bool enable = moduleActivation->getModuleStatus();
                ldbg << "ModuleActivationMsg recv - module: " << moduleName << ", active? " <<enable << endl;
            }
            else if(buddy->getContent() == BuddyMessage::AirMotion){ //This is only for airRobot!!!
                const AirDriveMessage *air = buddy->getAirDriveMessage();
                airDriver->onDriverMessage(*air);
            }
            if(buddy->getContent() == BuddyMessage::InformationTransfer){
                const InfoMessage *infoData = buddy->getInfoMessage();
                if(infoData->getInfoName() == FRAMERATE_SETTING_COMMAND){
                    upisController->setFPS(infoData->getInfo().toDouble());
                    ldbg << "fps of robot " << identifier << " setted to " << infoData->getInfo().toDouble() << endl;
                }
            }
        }
    }
}

void AirRobot::onUPISSimConnected()
{
    ldbg << "!!! UPIS CONNECTED !!!" << endl;
}

void AirRobot::onUPISDisconnected()
{
    ldbg << "!!! UPIS DISCONNECTED !!!" << endl;
}

void AirRobot::onUPISError(QString error)
{
    Q_UNUSED(error)
    //ldbg << "!!! UPIS ERROR: " << error << " !!!" << endl;
}

