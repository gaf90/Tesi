#include "robot.h"
#include "middleware/sensor/lasersensor.h"
//#define TESTING_GROUNDTRUTH
#ifndef TESTING_GROUNDTRUTH
#   include "middleware/sensor/odometrysensor.h"
#else
#   include "middleware/sensor/groundtruthsensor.h"
#endif
#include "middleware/sensor/camerasensor.h"
#include "middleware/sensor/statesensor.h"
#include "middleware/driver/wheeldriver.h"
#include "middleware/driver/wirelessdriver.h"
#include "middleware/sensor/sonarsensor.h"
#include "middleware/sensor/inssensor.h"
#include "data/usarmessage.h"
#include "data/wirelessmessage.h"
#include "shared/mutexqueue.h"
#include "shared/config.h"
#include "data/infomessage.h"
#include "data/buddymessage.h"
#include <typeinfo>
#include <QDebug>

using namespace Connection;
using namespace Data;
using namespace Middleware;
using namespace SLAM;

Robot::Robot(const QString &location, const QString &rotation, const uint identifier, bool isKenaf, QObject *parent) :
    QObject(parent),
    robotController(new RobotController(identifier, location, rotation,isKenaf)),
    usarController(new USARController()),
    upisController(new UPISController(2, identifier)),
    wssController(new WSSController(identifier)),
    sensors(new QList<Sensor *>()),
    drivers(new QList<Driver *>()),
    victimModule(new VictimDetection::VictimDetectionModule(identifier)),
    semMapModule(new SemanticMapping::SemanticMappingModule()),
    explorationModule(new Exploration::ExplorationModule(identifier, Config::policy)),
    coordinationModule(new Coordination::CoordinationModule(identifier)),
    pathPlannerModule(new PathPlanner::PathPlannerModule(identifier,isKenaf)), initialLocation(location),
    initialRotation(rotation), identifier(identifier), isKenaf(isKenaf)
{
    QStringList locationList = location.split(","), rotationList = rotation.split(",");

    ldbg << locationList << rotationList<<endl;

    slamModule = new SLAM::SLAMModule(identifier,
                                      Pose(locationList[1].toDouble(),
                                           locationList[0].toDouble(),
                                           -rotationList[2].toDouble()));
    robotController->setSlamModule(slamModule);

    ldbg<<"Robot: Add sensors and driver to the robot"<<endl;


    sensors->append(new LaserSensor());
    sensors->append(new INSSensor());
    sensors->append(new OdometrySensor());
    CameraSensor *cameraSensor = new CameraSensor();
    sensors->append(cameraSensor);

    connect(cameraSensor, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onSensorData(Data::Message)));
    sensors->append(new StateSensor());
    SonarSensor *sonarSensor = new SonarSensor();
    sensors->append(sonarSensor);

    drivers->append(new WheelDriver());
    wirelessDriver = new WirelessDriver(0, identifier);


    //Handle sensors' connection with controller
    for(int i=0; i<sensors->size(); i++)
    {
        //Usar Controller --> Sensor (Message received)
        connect(usarController, SIGNAL(sigMessage(Data::Message)), sensors->at(i), SLOT(onMessageReceived(Data::Message)));

        //Upis Controller --> Sensor (Message received)
        connect(upisController, SIGNAL(sigMessage(Data::Message)), sensors->at(i), SLOT(onMessageReceived(Data::Message)));

        //Sensor --> Robot Controller (Sensor data received)
        connect(sensors->at(i), SIGNAL(sigSensorData(Data::Message)), robotController, SLOT(onSensorData(Data::Message)), Qt::DirectConnection);

        //Sensor --> Slam (Sensor data received)
        connect(sensors->at(i), SIGNAL(sigSensorData(Data::Message)), slamModule, SLOT(onSensorData(Data::Message)), Qt::DirectConnection);

        //Sensor --> Robot (Sensor data received)
        connect(sensors->at(i), SIGNAL(sigSensorData(Data::Message)), this, SLOT(onSensorData(Data::Message)), Qt::DirectConnection);
    }

    //Handle drivers' connection with controller
    for(int i=0; i<drivers->size(); i++)
    {
        //Driver --> Usar Controller (Send driver message to USAR)
        connect(drivers->at(i), SIGNAL(sigDriveMessageSend(Data::Message)), usarController, SLOT(sendMessage(Data::Message)));

        //Robot Controller --> Driver (Send wheel message to driver)
        connect(robotController, SIGNAL(sigDriverMessageRobot(Data::Message)), drivers->at(i), SLOT(onDriverMessage(Data::Message)));
    }

    //Handle connection with the controller

    //Usar Controller --> Robot (Connect)
    connect(usarController, SIGNAL(sigConnected()), this, SLOT(onUSARSimConnected()));

    //Usar Controller --> Robot (Disconnect)
    connect(usarController, SIGNAL(sigDisconnected()), this, SLOT(onUSARSimDisconnected()));

    //Upis Controller --> Robot (Connect)
    connect(upisController, SIGNAL(sigConnected()), this, SLOT(onUPISSimConnected()));

    //Upis Controller --> Robot (Disconnect)
    connect(upisController, SIGNAL(sigDisconnected()), this, SLOT(onUPISDisconnected()));

    //Upis Controller --> Robot (Error)
    connect(upisController, SIGNAL(sigError(QString)), this, SLOT(onUPISError(QString)));

    /* WSS --> Wireless Driver */
    connect(wssController, SIGNAL(sigMessage(Data::Message)), wirelessDriver, SLOT(onMessageReceived(Data::Message)));   
    connect(wssController, SIGNAL(sigSendDistanceVector(Data::Message)),wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    connect(wssController, SIGNAL(sigInitializeConnection(QString)), wirelessDriver, SLOT(flushBuffer(QString)),Qt::DirectConnection);

    /* Wireless Driver --> WSS */
    connect(wirelessDriver, SIGNAL(sigDriveMessageSend(Data::Message)), wssController, SLOT(sendMessage(Data::Message)), Qt::DirectConnection);
    connect(wirelessDriver, SIGNAL(sigDistanceVector(QString,QHash<QString,int>)),wssController, SLOT(onDistanceVector(QString,QHash<QString,int>)));

    // Wireless Driver --> Robot Controller
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), robotController, SLOT(onSensorData(Data::Message)));
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onSensorData(Data::Message)));

    /* Robot --> WirelessDriver */
    connect(robotController, SIGNAL(sigWirelessMessageRCM(Data::Message)), wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    connect(this, SIGNAL(sigWaypointReceived(const Data::WaypointCommand*)), robotController, SLOT(handleWaypoint(const Data::WaypointCommand*)));

    connect(slamModule, SIGNAL(newRobotPose(SLAM::TimedPose)), robotController, SLOT(onNewRobotPose(SLAM::TimedPose)), Qt::DirectConnection);

    /* Connection of the path planning module */
    pathPlannerModule->setSlamModule(slamModule);

    connect(explorationModule, SIGNAL(sigPerformActionEM(PathPlanner::AbstractAction *)), robotController, SLOT(onPerformActionRCM(PathPlanner::AbstractAction *)), Qt::DirectConnection);
    connect(explorationModule, SIGNAL(sigStopRobotForPlanningEM()), robotController, SLOT(onStopRobotForPlanning()));
    connect(explorationModule, SIGNAL(sigRestartExplorationEM()), robotController, SLOT(onRestartExploration()));
    connect(explorationModule, SIGNAL(sigFrontierToReachEM(Data::Pose)), robotController, SLOT(onFrontierToReachRCM(Data::Pose)),Qt::DirectConnection);
    //Initialize the module of the robot

    explorationModule->connect(explorationModule, SIGNAL(sigDriverMessageEM(Data::Message)), wirelessDriver, SLOT(onDriverMessage(Data::Message)), Qt::DirectConnection);
    explorationModule->connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), explorationModule, SLOT(onUpdateSignalStrength(Data::Message)), Qt::DirectConnection);

    connect(wssController,SIGNAL(sigUpdateSignalStrength(QString)),this,SLOT(onUpdateSignalStrength(QString)));

    explorationModule->setSLAMModule(slamModule);
    explorationModule->setSemModule(semMapModule);
    explorationModule->setCoordModule(coordinationModule);
    explorationModule->setRobotState(robotController->getActualState());
    explorationModule->setStartingValues();
    explorationModule->start();

    connect(robotController, SIGNAL(sigChangeStateExplorationRCM(bool)), explorationModule, SLOT(changeStatus(bool)));
    connect(this, SIGNAL(sigEnableUserCriteria(bool,const Data::HighLevelCommand*)),
            explorationModule, SLOT(onEnableUserCriterionSignal(bool,const Data::HighLevelCommand*)));
    connect(robotController, SIGNAL(sigChangeStatePathPlanningRCM(bool)), pathPlannerModule, SLOT(changeStatus(bool)));
    victimModule->start();
    connect(explorationModule, SIGNAL(sigNoFrontierAvailableEM()), robotController, SLOT(onNoFrontierAvailableRCM()));
    //BAD FRONTIERS
    connect(pathPlannerModule, SIGNAL(sigHandleBandleFrontierPM(Data::Pose)), explorationModule, SLOT(handleBadFrontier(Data::Pose)), Qt::DirectConnection);
    connect(robotController, SIGNAL(sigHandleBadFrontierRCM(Data::Pose)), explorationModule, SLOT(handleBadFrontier(Data::Pose)),Qt::DirectConnection);
    connect(pathPlannerModule, SIGNAL(sigCleanBadFrontiersPM()), explorationModule, SLOT(cleanBadFrontiers()), Qt::DirectConnection);
    connect(robotController, SIGNAL(sigCleanBadFrontierRCM()), explorationModule, SLOT(cleanBadFrontiers()), Qt::DirectConnection);
    connect(pathPlannerModule,SIGNAL(sigHandleBandleFrontierPM(Data::Pose)),robotController, SLOT(onHandleBadFrontierRCM(Data::Pose)),Qt::DirectConnection);

    //##COORDINATION SIGNALS/SLOT CONNECTIONS AND OTHERS##//
    coordinationModule->setExplorationModule(explorationModule);
    coordinationModule->setCoordinationMessageQueue(new Shared::MutexQueue<const Data::BuddyMessage *>());
    coordinationModule->setExplorationModule(explorationModule);
    coordinationModule->setPathPlannerModule(pathPlannerModule);
    //forwarding wireless messages coming from the coordination module to the wireless module
    connect(coordinationModule, SIGNAL(sigMessageSendCM(Data::Message)), wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    //analysing all incoming messages, the one related to the coordination are forwarded to the module
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), this, SLOT(onCoordinationMessage(Data::Message)));
    connect(wirelessDriver, SIGNAL(sigSensorData(Data::Message)), slamModule, SLOT(onSensorData(Data::Message)), Qt::DirectConnection);

    connect(&plasterMapper, SIGNAL(timeout()), this, SLOT(handlePlasterMapper()));

    plasterMapper.start(2000);

    connect(wssController,SIGNAL(sigRobotNotReachable()),robotController,SLOT(onRobotNotReachable()));

    //VICTIM DETECTION MODULE
    connect(robotController, SIGNAL(sigCameraDataRCM(Data::CameraData, Data::Pose)), victimModule, SLOT(onCameraDataSignal(Data::CameraData, Data::Pose)));
    connect(victimModule, SIGNAL(sendMessageSignal(Data::Message)), wirelessDriver, SLOT(onDriverMessage(Data::Message)));
    connect(this, SIGNAL(signalVictimDetectionConfiguration(double,double,double,double,double,double)),
            victimModule, SLOT(updateHSVConfig(double,double,double,double,double,double)));

    //OBSTACLE AVOIDANCE

         connect(robotController, SIGNAL(sigRecomputePathRCM(Data::Pose)),explorationModule, SIGNAL(sigFrontierToReachEM(Data::Pose)));

    //robotController->setUserEnabled(true);
}

Robot::~Robot()
{   
    delete coordinationModule;
    delete explorationModule;
    delete semMapModule;
    delete slamModule;
    delete victimModule;
    //delete graphics;
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

void Robot::spawnRobot()
{
    //connect the robot to USARSim
    ldbg << "Robot: Usar Address:Port " << Config::usarAddress<< ":"<<Config::usarPort<<endl;
    usarController->connectToHost(Config::usarAddress, Config::usarPort);

    //Connect to upis
    ldbg << "Robot: Upis Address:Port " << Config::upisAddress<< ":"<<Config::upisPort<<endl;
    upisController->connectToHost(Config::upisAddress, Config::upisPort);

    //Connect to WSS
    ldbg << "Robot: WSS Address:Port " << Config::wssAddress<< ":"<<Config::wssPort<<endl;
    wssController->connectToHost(Config::wssAddress, Config::wssPort);

}

void Robot::onUSARSimConnected()
{
    //Spawn the robot in the simulation
    USARMessage message;
    message.setType("INIT");
    if(isKenaf)
        message.insert("ClassName", "USARBot.kenaf");
    else
        message.insert("ClassName", "USARBot.P3AT");
    message.insert("Name", robotNameFromIndex(identifier));
    message.insert("Location", initialLocation);
    message.insert("Rotation", initialRotation);

    usarController->sendMessage(message);


}

void Robot::onUSARSimDisconnected()
{
    usarController->connectToHost(Config::usarAddress, Config::usarPort);
}



void Robot::onCoordinationMessage(const Data::Message &msg)
{
    if(typeid(msg) == typeid(const WirelessMessage &)){
        const WirelessMessage &mess = (const WirelessMessage &)msg;
        if(mess.getCommand() == WirelessMessage::MessageExchange){
            const BuddyMessage *buddy = mess.getBuddyMessage();
            if(buddy->getContent() == BuddyMessage::AskBidsOnFrontiers ||
                    buddy->getContent() == BuddyMessage::SendBidsOnFrontiers ||
                    buddy->getContent() == BuddyMessage::FrontierAssignment ||
                    buddy->getContent() == BuddyMessage::AskBidsOnVictims ||
                    buddy->getContent() == BuddyMessage::SendBidsOnVictims ||
                    buddy->getContent() == BuddyMessage::VictimAssignment ||
                    buddy->getContent() == BuddyMessage::AcknowledgmentMessage ||
                    buddy->getContent() == BuddyMessage::VictimConfirmation ||
                    buddy->getContent() == BuddyMessage::VictimDeletion){
                coordinationModule->getCoordinationMessageQueue()->enqueue(new BuddyMessage(*buddy));
            }
        }
    }
}


void Robot::onSensorData(const Message &message)
{
    if(typeid(message) == typeid(const WirelessMessage &))
    {
        const WirelessMessage &wirelessMessage = (const WirelessMessage &)message;
        if (wirelessMessage.getCommand() == WirelessMessage::MessageExchange)
        {
            const BuddyMessage *buddy = wirelessMessage.getBuddyMessage();

            ldbg << "Robot: Message Type " << buddy->getContent() << endl;


            if(buddy->getContent() == BuddyMessage::ModuleActivation)
            {
                handleModuleActivationMessage(buddy);
            }

            else if (buddy->getContent() == BuddyMessage::WaypointMessage)
            {
                robotController->wayPointCounter = 0;
                handleWaypointMessage(buddy);
            }

            else if(buddy->getContent() == BuddyMessage::VictimDetectionConfiguration)
            {
                handleVictimDetectionConf(buddy);
            }

            else if (buddy->getContent() == BuddyMessage::HighCommand)
            {
                const HighLevelCommand* command = buddy->get<HighLevelCommand>();
                emit sigEnableUserCriteria(true, command);

            }

            else if(buddy->getContent() == BuddyMessage::WheelMotion)
            {
                emit sigEnableUserCriteria(false, NULL);

            }

            else if(buddy->getContent() == BuddyMessage::InformationTransfer)
            {
                const InfoMessage *infoData = buddy->getInfoMessage();
                handleInfoMessage(infoData);
            }
        }
    }

}

void Robot::handleModuleActivationMessage(const BuddyMessage *buddy)
{
    const ModuleActivationMessage* moduleActivation = buddy->get<ModuleActivationMessage>();
    QString moduleName = moduleActivation->getInvolvedModule();
    bool moduleStatus = moduleActivation->getModuleStatus();

    ldbg << "Robot: Module Activation Message - module: " << moduleName << ", active? " << moduleStatus << endl;

    changeModuleStatus(moduleName, moduleStatus);

    if (moduleName.compare("exploration") == 0)
    {
        robotController->isExplorationEnabled = moduleStatus;

        if (moduleStatus)
            robotController->sonarStatus = 0;
        else
        {
            robotController->obstacleAvoidance->empiricBehaviorStatus =0;
            robotController->obstacleAvoidance->neuralBehaviorStatus = 0;
            robotController->obstacleAvoidance->dwaBehaviorStatus = 0;
            robotController->sonarStatus = 1;
            robotController->stopRobot(true);
        }
    }

}

void Robot::handleWaypointMessage(const BuddyMessage *buddy)
{
    ldbg << "Robot: Waypoint Message " << endl;

    const WaypointCommand* waypoint = buddy->get<WaypointCommand>();
    explorationModule->changeStatus(false);
    pathPlannerModule->changeStatus(false);


    emit sigWaypointReceived(waypoint);
    emit sigEnableUserCriteria(false, NULL);
}

void Robot::handleVictimDetectionConf(const BuddyMessage *buddy)
{
    ldbg << "Robot: Victim Detection Message " << endl;

    const VictimDetectionConfMessage* conf = buddy->get<VictimDetectionConfMessage>();
    emit signalVictimDetectionConfiguration(conf->getHMin(), conf->getSMin(), conf->getVMin(),
                                            conf->getHMax(), conf->getSMax(), conf->getVMax());
}

void Robot::handleInfoMessage(const InfoMessage *infoData)
{
    ldbg << "Robot: Information Message " << endl;

    if(infoData->getInfoName() == FRAMERATE_SETTING_COMMAND)
    {
        double fps = infoData->getInfo().toDouble();
        upisController->setFPS(fps);
        victimModule->setFPS(fps);

        ldbg << "Robot: Fps of robot " << identifier << " setted to " << infoData->getInfo().toDouble() << endl;
    }
}

void Robot::changeModuleStatus(const QString &moduleName, bool status)
{
    if(moduleName == QString(EXPLORATION))
    {
        ldbg << "Robot: Change Exploration Status " << endl;
        robotController->setStatus(status);
    }
    if(moduleName == QString(VICTIM_DETECTION))
    {
        ldbg << "Robot: Change Victim Status " << endl;
        victimModule->setStatus(status);
    }
    if(moduleName == QString(POARET_SLAM_MODULE))
    {
        ldbg << "Robot: Change SLAM Status " << endl;
        if(status == false)
        {
            ldbg << "Robot: Disconnecting coord from pathplanning" << endl;
            disconnect(coordinationModule, SIGNAL(sigPointToReachCM(double,double)), pathPlannerModule, SLOT(onPointToReachPP(double,double)));

            ldbg << "Robot: Connecting coord to robotController" << endl;
            connect(coordinationModule, SIGNAL(sigPointToReachCM(double,double)), robotController, SLOT(onPointToReachRCM(double,double)));

        }
    }
}

void Robot::handlePlasterMapper()
{
    SLAM::Map map = slamModule->getMap();
    BuddyMessage buddy(robotNameFromIndex(identifier), robotNameFromIndex(BASE_STATION_ID),
                       BuddyMessage::MapInformation, &map);
    WirelessMessage msg(&buddy);    
    wirelessDriver->onDriverMessage(msg);
}

void Robot::onPowerSignalDataGathered()
{
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

    ldbg << "Robot: Power data = "<<data<<endl;

    InfoMessage info(CONNECTION_INFORMATION, data);
    BuddyMessage buddy(robotNameFromIndex(identifier), robotNameFromIndex(BASE_STATION_ID), &info);
    WirelessMessage ws(&buddy);
    wirelessDriver->onDriverMessage(ws);
    powerTable->clear();
    delete powerTable;
}

void Robot::onUPISSimConnected()
{
    ldbg << "Robot: Upis connected" << endl;
}

void Robot::onUPISDisconnected()
{
    ldbg << "Robot: Upis disconnected" << endl;
}

void Robot::onUPISError(QString error)
{
    ldbg << "Robot: Upis error code " << error << endl;
}

void Robot::onUpdateSignalStrength(QString strength)
{
    InfoMessage info(CONNECTION_INFORMATION, strength);

    ldbg << strength <<endl;

    if (strength.toDouble() < -90.0)
    {
        ldbg << "Robot: Error 404 Not found"<<endl;
        robotController->obstacleAvoidance->empiricBehaviorStatus =0;
        robotController->sonarStatus = 1;
        robotController->stopRobot(true);
        robotController->isExplorationEnabled = false;
        robotController->setStatus(false);
    }

    BuddyMessage buddy(robotNameFromIndex(identifier), robotNameFromIndex(BASE_STATION_ID), &info);
    WirelessMessage ws(&buddy);
    wirelessDriver->onDriverMessage(ws);
}

