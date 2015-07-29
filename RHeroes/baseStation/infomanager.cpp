#include "infomanager.h"
#include "shared/utilities.h"
#include "shared/constants.h"
#include <QWidget>

#define MAX_BATTERY 1200
//#define BS_IM_EXPLORATION "exploration"
//#define BS_IM_GLOBAL "global"
//#define BS_IM_VICTIM_DETECTION "victim"
//#define BS_IM_SLAM "slam"

namespace BaseStation{

InfoManager::InfoManager(QObject *parent, uint nRobots) :
    QObject(parent), knownRobots(new QHash<uint, t_singleRobot>),
    singleRobotInfoView(new graphics::SingleRobotInfoWidget()),
    selectedRobot(0), nRobots(nRobots)
{
    connect(singleRobotInfoView, SIGNAL(sigChangeModuleStatusSIW(QString,bool)),
            this, SLOT(onModuleActivation(QString,bool)));
    connect(singleRobotInfoView,SIGNAL(signalBrightnessChanged(int)), this, SLOT(onBrightnessChanged(int)));
    connect(singleRobotInfoView,SIGNAL(signalContrastChanged(int)), this, SLOT(onContrastChanged(int)));
    connect (singleRobotInfoView, SIGNAL(signalAutoModeOn()), this, SLOT(onAutoModeOn()));
    connect (singleRobotInfoView, SIGNAL(signalRobotNearVictim()), this, SLOT(onRobotNearVictim()));

    //Initialization of the brightness and contrast values of robot cameras:
    for(uint i=0; i<nRobots ; i++){
        brightnessValues[i]= 0;
        contrastValues[i]= 0;
        robotNearVictim[i]= false;
    }
}

graphics::SingleRobotInfoWidget *InfoManager::getSingleRobotInfoWidget()
{
    return this->singleRobotInfoView;
}

void InfoManager::onSelectedRobotChangedIM(uint id)
{
    if(selectedRobot == id)
        return;
    selectedRobot = id;
    requestInformation();
    singleRobotInfoView->setRobot(robotNameFromIndex(selectedRobot));
    showInformation(false);
}


void InfoManager::onNewInfoMessageReceived(Data::InfoMessage message, uint robot)
{
    QString info = message.getInfoName();
    t_singleRobot data = {999,-1,-1,-1,0, 0, "none",-1};
    data = knownRobots->value(robot, data);
    if(data.robotID == 999){
        t_singleRobot data2 = {robot, 100, 0, 0, new QHash<uint, double>(), new QHash<QString, bool>(), "none",1};
        data = data2;
        data.moduleActivations->insert(EXPLORATION, false);
        data.moduleActivations->insert(SEMANTIC_MAPPING, true);
        data.moduleActivations->insert(VICTIM_DETECTION, true);
        knownRobots->insert(robot, data);
    }
    bool updated = false;
    if(info == BATTERY_STATUS){
        double newBattery = message.getInfo().toInt();
        if(data.batteryStatus != newBattery && newBattery >= 0)
        {
            data.batteryStatus = (1 - (MAX_BATTERY - newBattery)/MAX_BATTERY) * 100;
            updated = true;
            knownRobots->insert(robot, data);
        }
    }
    if(info == CONNECTION_INFORMATION){
        //eg: "1=14.2;2=98.0"
        double power=message.getInfo().toDouble();
        double exp=((double)90+power);
        double perc=exp;
        data.wssConnections->insert(robot, perc);
        updated = true;
        data.signalStrenght=perc;
        knownRobots->insert(robot, data);

        /*
        QStringList infoTot = message.getInfo().split(";");
        QStringList info;
        uint destinationId;
        for(int i = 0; i< infoTot.size(); i++)
        {
            info = infoTot.at(i).split("=");
            destinationId = info.at(0).toUInt();
            if(destinationId <= BASE_STATION_ID)
            {
                double power = info.at(1).toDouble();
                if(power >= 0)
                {
                    data.wssConnections->insert(destinationId, power);
                    updated = true;
                    knownRobots->insert(robot, data);
                }
            }
            else{
                return;
            }
        }*/
    }
    if(updated)
        showInformation(true);
}

void InfoManager::onChangeModuleStatusIM(QString module, bool enabled)
{
    for(uint robot = 0; robot < nRobots; robot++)
    {
        t_singleRobot data = {999,-1,-1,-1,0, 0, "none",-1};
        data = knownRobots->value(robot, data);
        if(data.robotID == 999){
            t_singleRobot data2 = {robot, 100, 0, 0, new QHash<uint, double>(), new QHash<QString, bool>(), "none",1};
            data = data2;
            data.moduleActivations->insert(EXPLORATION, false);
            data.moduleActivations->insert(SEMANTIC_MAPPING, true);
            data.moduleActivations->insert(VICTIM_DETECTION, true);
            knownRobots->insert(robot, data);
        }
        data.moduleActivations->insert(module, enabled);
        knownRobots->insert(robot, data);
    }
    showInformation(false);
}

void InfoManager::onModuleActivation(QString module, bool enabled)
{
    t_singleRobot data = {999,-1,-1,-1,0, 0, "none",-1};
    data = knownRobots->value(selectedRobot, data);
    if(data.robotID == 999){
        t_singleRobot data2 = {selectedRobot, 100, 0, 0, new QHash<uint, double>(), new QHash<QString, bool>(), "none",1};
        data = data2;
        data.moduleActivations->insert(EXPLORATION, false);
        data.moduleActivations->insert(SEMANTIC_MAPPING, true);
        data.moduleActivations->insert(VICTIM_DETECTION, true);
        knownRobots->insert(selectedRobot, data);
    }
    data.moduleActivations->insert(module, enabled);
    knownRobots->insert(selectedRobot, data);
    emit sigChangeModuleStatusIM(module, enabled, this->selectedRobot);
}
void InfoManager::onBrightnessChanged(int newValue){
    brightnessValues[this->selectedRobot]= newValue;
    singleRobotInfoView->setBrightnessValue(brightnessValues[selectedRobot]);
    showInformation(false);
    emit signalBrightnessChanged(newValue, this->selectedRobot);
}

void InfoManager::onContrastChanged(int newValue){
    contrastValues[this->selectedRobot]= newValue;
    singleRobotInfoView->setContrastValue(contrastValues[selectedRobot]);
    showInformation(false);
    emit signalContrastChanged(newValue, this->selectedRobot);
}

void InfoManager::onAutoModeOn(){
    // enable autonomous exploration for all the robots
    for(int i=0; i<=nRobots; i++){
        if(robotNearVictim[i]== false){
        t_singleRobot data = {999,-1,-1,-1,0, 0, "none",-1};
        data = knownRobots->value(i, data);
        if(data.robotID == 999){
            t_singleRobot data2 = {i, 100, 0, 0, new QHash<uint, double>(), new QHash<QString, bool>(), "none",1};
            data = data2;
            data.moduleActivations->insert(EXPLORATION, false);
            data.moduleActivations->insert(SEMANTIC_MAPPING, true);
            data.moduleActivations->insert(VICTIM_DETECTION, true);
            knownRobots->insert(i, data);
        }
        data.moduleActivations->insert(EXPLORATION, true);
        knownRobots->insert(i, data);
        showInformation(false);
        emit sigChangeModuleStatusIM(EXPLORATION, true, i);
    }
    }
}
void InfoManager::onRobotNearVictim(){
    robotNearVictim[this->selectedRobot]=true;
}

void InfoManager::requestInformation()
{
    emit sigSendInfoMessage(selectedRobot);
}

void InfoManager::showInformation(bool interacting)
{
    t_singleRobot data = {999,-1,-1,-1,0, 0, "none",-1};
    data = knownRobots->value(selectedRobot, data);
    if(data.robotID != 999){
        singleRobotInfoView->setBatteryStatus(data.batteryStatus);
        singleRobotInfoView->setBrightnessValue(brightnessValues[selectedRobot]);
        singleRobotInfoView->setContrastValue(contrastValues[selectedRobot]);
        singleRobotInfoView->clearWirelessTable();
        singleRobotInfoView->setSignalStrenghtStatus(data.signalStrenght);
        foreach(uint robot, data.wssConnections->keys()){
            singleRobotInfoView->addWirelessInformation(robotNameFromIndex(robot),data.wssConnections->value(robot));
        }
        bool globalStop = !data.moduleActivations->value(EXPLORATION) &&
                !data.moduleActivations->value(SEMANTIC_MAPPING) &&
                !data.moduleActivations->value(VICTIM_DETECTION);
        if(!interacting)
            singleRobotInfoView->setActivationStatus(globalStop,
                    data.moduleActivations->value(EXPLORATION),
                    data.moduleActivations->value(SEMANTIC_MAPPING),
                    data.moduleActivations->value(VICTIM_DETECTION) );
        if(globalStop)
        {
            data.mission = "Stop and wait here";
            knownRobots->insert(selectedRobot, data);
        }
        singleRobotInfoView->setMission(data.mission);
    }
    else
    {
        singleRobotInfoView->setActivationStatus(false, false, true, true);
        singleRobotInfoView->setMission("none");
    }
}

}
