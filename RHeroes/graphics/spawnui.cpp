#include "spawnui.h"
#include "ui_spawnui.h"
#include "QDebug"
#include <typeinfo>
#include "baseStation/graphicparams.h"
#include "data/usarmessage.h"
#include "shared/config.h"
#include <QDebug>

#define INITIAL_ROBOT_ID 0

namespace graphics
{

SpawnUI::SpawnUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SpawnUI),
    locations(new QHash<QString,QStringList>()),
    error(new QErrorMessage(this)), bsSpawned(false), spawnedRobots(new QHash<uint, t_spawnInfo>())
{
    ui->setupUi(this);

/*---------------------------------------------------------------------------------------------------------
                    initial parameters from poaret.conf, configuration file
---------------------------------------------------------------------------------------------------------*/

    ui->wssAddressLine->setText(Config::wssAddress);
    ui->upisAddressLine->setText(Config::upisAddress);
    ui->usarAddressLine->setText(Config::usarAddress);
    ui->wssPortLine->setText(QString::number(Config::wssPort));
    ui->upisPortLine->setText(QString::number(Config::upisPort));
    ui->usarPortLine->setText(QString::number(Config::usarPort));
    ui->maxRobots->setValue(Config::robotCount);
    ui->bsXLineEdit->setText(QString::number(Config::baseStationPose.getX()));
    ui->bsYLineEdit->setText(QString::number(Config::baseStationPose.getY()));
    ui->bsZLineEdit->setText("0");

//    -----------------------------------------------------------------------------------------------------

    connect(ui->startButton,SIGNAL(clicked()), this, SLOT(onStartApplication()));
    connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(onConnectButtonClicked()));
    connect(ui->spawnButton, SIGNAL(clicked()), this, SLOT(onSpawnButtonClicked()));
    connect(ui->airRobotSpawn, SIGNAL(clicked()), this, SLOT(onAirSpawnButtonClicked()));
    connect(ui->configureButton, SIGNAL(clicked()), this, SLOT(onConfigureButtonClicked()));
    connect(ui->robotLocationBox, SIGNAL(currentIndexChanged(QString)), this, SLOT(onInitialLocationSelected(QString)));
    connect(ui->spawnBSButton, SIGNAL(clicked()), this, SLOT(onSpawnBS()));
    connect(ui->kenafSpawnButton, SIGNAL(clicked()), this, SLOT(onKenafSpawnButtonClicked()));

    connect(ui->respawnButton, SIGNAL(clicked()), this, SLOT(onRespawnButtonClicked()));

    ui->nSpawnedRobots->display(INITIAL_ROBOT_ID);
}

SpawnUI::~SpawnUI()
{
    delete ui;
    delete locations;
    delete error;
}

void SpawnUI::onInitialLocation(const QString &location)
{
    ui->console->append(location);
    QStringList position = location.split(",");
    locations->insert(position[0], position);
    ui->robotLocationBox->addItem(position[0]);
}

void SpawnUI::onInitialLocationSelected(QString loc)
{
    QStringList position = locations->value(loc);
    ui->xLineEdit->setText(position[1]);
    ui->yLineEdit->setText(position[2]);
    ui->zLineEdit->setText(position[3]);
    ui->roLineEdit->setText(position[4]);
    ui->fiLineEdit->setText(position[5]);
    ui->thetaLineEdit->setText(position[6]);
    if(!bsSpawned)
    {
        ui->bsXLineEdit->setText(position[1]);
        ui->bsYLineEdit->setText(position[2]);
        ui->bsZLineEdit->setText(position[3]);
    }
}



void SpawnUI::appendOnConsole(const Data::Message &message)
{
    if(typeid(message) == typeid(Data::USARMessage)){
        const Data::USARMessage &text = (const Data::USARMessage &)message;
        ui->console->append(text.operator QString());
    }
}

void SpawnUI::appendOnConsole(const QString &message)
{
    ui->console->append(message);
}

void SpawnUI::onConnectButtonClicked()
{
    if(ui->connectButton->text().compare(CONNECT) == 0){
        //Collect data to install a connection
        QString usarAddress = ui->usarAddressLine->text();
        uint usarPort = ui->usarPortLine->text().toInt();
        QString upisAddress = ui->upisAddressLine->text();
        uint upisPort = ui->upisPortLine->text().toInt();
        QString wssAddress = ui->wssAddressLine->text();
        uint wssPort = ui->wssPortLine->text().toInt();
        emit sigUSARConnect(usarAddress, usarPort);
        emit siglUPISConnect(upisAddress, upisPort);
        //onSpawnBS();
        emit siglWSSConnect(wssAddress, wssPort);
    } else if (ui->connectButton->text().compare(DISCONNECT) == 0){
        emit sigDisconnect();
    }
}

void SpawnUI::onSpawnButtonClicked()
{
    if(ui->maxRobots->value()>ui->nSpawnedRobots->intValue()){
        //Collect data to spawn a new robot from the ui.
        QString location = ui->xLineEdit->text() + "," +
                ui->yLineEdit->text() + "," + ui->zLineEdit->text();
        QString rotation = ui->roLineEdit->text() + "," +
                ui->fiLineEdit->text() + "," + ui->thetaLineEdit->text();
        QString bsLocation = ui->bsXLineEdit->text() + "," + ui->bsYLineEdit->text() + "," +
                ui->bsZLineEdit->text();

        t_spawnInfo info = {"", "", "", 0, 0, false, false};
        info.bsLocation = bsLocation;
        info.location = location;
        info.rotation = rotation;
        info.aerial = false;
        info.nActiveRobot = ui->nSpawnedRobots->intValue();
        info.maxRobots = ui->maxRobots->value();
        spawnedRobots->insert(info.nActiveRobot, info);

        emit sigSpawnRobotSU(location, rotation, bsLocation, ui->nSpawnedRobots->intValue(),
                              ui->maxRobots->value(), false, false);
        ui->nSpawnedRobots->display(ui->nSpawnedRobots->intValue()+1);
        if(ui->nSpawnedRobots->intValue() == ui->maxRobots->value())
            ui->startButton->setEnabled(true);
    }
    else
    {
        error->showMessage("You already spawned the maximum number of robots! You should have modified Max Robots earlier!");
    }
}

void SpawnUI::onAirSpawnButtonClicked()
{
    if(ui->maxRobots->value()>ui->nSpawnedRobots->intValue()){
        //Collect data to spawn a new robot from the ui.
        QString location = ui->xLineEdit->text() + "," +
                ui->yLineEdit->text() + "," + ui->zLineEdit->text();
        QString rotation = ui->roLineEdit->text() + "," +
                ui->fiLineEdit->text() + "," + ui->thetaLineEdit->text();
        QString bsLocation = ui->bsXLineEdit->text() + "," + ui->bsYLineEdit->text() + "," +
                ui->bsZLineEdit->text();

        t_spawnInfo info = {"", "", "", 0, 0, false, false};
        info.bsLocation = bsLocation;
        info.location = location;
        info.rotation = rotation;
        info.aerial = true;
        info.nActiveRobot = ui->nSpawnedRobots->intValue();
        info.maxRobots = ui->maxRobots->value();
        spawnedRobots->insert(info.nActiveRobot, info);

        emit sigSpawnRobotSU(location, rotation, bsLocation, ui->nSpawnedRobots->intValue(),
                              ui->maxRobots->value(), true, false);
        ui->nSpawnedRobots->display(ui->nSpawnedRobots->intValue()+1);
        if(ui->nSpawnedRobots->intValue() == ui->maxRobots->value())
            ui->startButton->setEnabled(true);
    }
    else
    {
        error->showMessage("You already spawned the maximum number of robots! You should have modified Max Robots earlier!");
    }
}

void SpawnUI::onKenafSpawnButtonClicked()
{
    if(ui->maxRobots->value()>ui->nSpawnedRobots->intValue()){
        //Collect data to spawn a new robot from the ui.
        QString location = ui->xLineEdit->text() + "," +
                ui->yLineEdit->text() + "," + ui->zLineEdit->text();
        QString rotation = ui->roLineEdit->text() + "," +
                ui->fiLineEdit->text() + "," + ui->thetaLineEdit->text();
        QString bsLocation = ui->bsXLineEdit->text() + "," + ui->bsYLineEdit->text() + "," +
                ui->bsZLineEdit->text();

        t_spawnInfo info = {"", "", "", 0, 0, false, false};
        info.bsLocation = bsLocation;
        info.location = location;
        info.rotation = rotation;
        info.aerial = false;
        info.kenaf = true;
        info.nActiveRobot = ui->nSpawnedRobots->intValue();
        info.maxRobots = ui->maxRobots->value();
        spawnedRobots->insert(info.nActiveRobot, info);

        emit sigSpawnRobotSU(location, rotation, bsLocation, ui->nSpawnedRobots->intValue(),
                              ui->maxRobots->value(), false, true);
        ui->nSpawnedRobots->display(ui->nSpawnedRobots->intValue()+1);
        if(ui->nSpawnedRobots->intValue() == ui->maxRobots->value())
            ui->startButton->setEnabled(true);
    }
    else
    {
        error->showMessage("You already spawned the maximum number of robots! You should have modified Max Robots earlier!");
    }
}

void SpawnUI::onSendRawButtonClicked()
{
    QString rawCommand = ui->rawCmdLine->text();
    Data::USARMessage message(rawCommand);
    emit signalRawMessage(message);
}

void SpawnUI::enableComponents(bool enable)
{
    if(enable == FALSE){
        ui->connectButton->setText(DISCONNECT);
        ui->statusLabel->setText("Connected to USARSim");
    } else {
        ui->connectButton->setText(CONNECT);
        ui->statusLabel->setText("Disconnected");
    }
    ui->usarAddressLine->setEnabled(enable);
    ui->usarPortLine->setEnabled(enable);
    ui->upisAddressLine->setEnabled(enable);
    ui->upisPortLine->setEnabled(enable);
    ui->wssAddressLine->setEnabled(enable);
    ui->wssPortLine->setEnabled(enable);

    ui->rawCmdLine->setEnabled(!enable);
    ui->sendRawButton->setEnabled(!enable);
    ui->spawnButton->setEnabled(!enable);
    ui->airRobotSpawn->setEnabled(!enable);
    ui->kenafSpawnButton->setEnabled(!enable);
    ui->robotLocationBox->setEnabled(!enable);

}

void SpawnUI::onDisconnected()
{
    enableComponents(true);
    ui->console->append("Disconnected");
    for(int i = 0; i < ui->robotLocationBox->count(); i++)
        ui->robotLocationBox->removeItem(i);
    locations->clear();
}

void SpawnUI::onModuleConnected()
{
    enableComponents(false);
    ui->console->append("Connected to UsarSim");
}

void SpawnUI::onStartApplication()
{
    emit sigApplicationStarted(ui->maxRobots->value());
}

void SpawnUI::onConfigureButtonClicked()
{   //disabling all configuration params.
    emit sigApplicationConfigure(ui->maxRobots->value());
    ui->maxRobots->setEnabled(false);
    ui->configureButton->setEnabled(false);
    ui->connectButton->setEnabled(true);
    ui->startButton->setEnabled(true);
}

void SpawnUI::onSpawnBS()
{
    if(ui->connectButton->text() != DISCONNECT)
        return;
    ui->bsXLineEdit->setEnabled(false);
    ui->bsYLineEdit->setEnabled(false);
    ui->bsZLineEdit->setEnabled(false);
    ui->spawnBSButton->setEnabled(false);
    QString loc = ui->bsXLineEdit->text() + "," + ui->bsYLineEdit->text() + "," +
            ui->bsZLineEdit->text();
    bsSpawned = true;
    emit signalSpawnBS(loc);
}

void SpawnUI::onRespawnButtonClicked()
{
    t_spawnInfo info = spawnedRobots->value(ui->respawnSpinBox->value());
    emit sigRespawnRobot(info.location, info.rotation, info.bsLocation, info.nActiveRobot,
                            info.maxRobots, info.aerial, info.kenaf);
}


QString SpawnUI::getBSLocation()
{
    QString loc = ui->bsXLineEdit->text() + "," + ui->bsYLineEdit->text() + "," +
            ui->bsZLineEdit->text();
    return loc;
}

}



