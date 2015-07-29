#include "basestationui.h"
#include "ui_basestationui.h"
#include "robotui.h"
#include "data/usarmessage.h"
#include <typeinfo>
#include <QDebug>

using namespace Data;

BaseStationUi::BaseStationUi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BaseStationUi())
{
    //Setup the graphical environment
    ui->setupUi(this);
    //Connection to handle the UI events.
    connect(ui->connectButton, SIGNAL(clicked()), this, SLOT(onConnectButtonClicked()));
    connect(ui->sendRawButton, SIGNAL(clicked()), this, SLOT(onSendRawButtonClicked()));
    connect(ui->spawnButton, SIGNAL(clicked()), this, SLOT(onSpawnButtonClicked()));
}

BaseStationUi::~BaseStationUi()
{
    delete ui;
}

void BaseStationUi::onConnectButtonClicked()
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
        emit siglWSSConnect(wssAddress, wssPort);
    } else if (ui->connectButton->text().compare(DISCONNECT) == 0){
        emit sigDisconnect();
    }
}

void BaseStationUi::onSendRawButtonClicked()
{
    //Collect data to send to USARSim
    QString rawCommand = ui->rawCmdLine->text();
    USARMessage message(rawCommand);
    emit sigMessage(message);
}

void BaseStationUi::onSpawnButtonClicked()
{
    //Collect data to spawn a new robot from the ui.
    QString location = (ui->robotLocationBox->currentText());
    emit sigSpawnRobotBU(location);
}

void BaseStationUi::appendOnConsole(const QString &message)
{
    ui->console->append(message);
}

void BaseStationUi::appendOnConsole(const Message &message)
{
    if(typeid(message) == typeid(USARMessage)){
        const USARMessage &text = (const USARMessage &)message;
        ui->console->append(text.operator QString());
    }

}

void BaseStationUi::onModuleConnected()
{
    enableComponents(false);
    ui->console->append("Connected to UsarSim");
}

void BaseStationUi::onDisconnected(){
    enableComponents(true);
    ui->console->append("Disonnected");
}

void BaseStationUi::enableComponents(bool enable)
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
    ui->robotLocationBox->setEnabled(!enable);

}

void BaseStationUi::onInitialLocation(const QString &location)
{
    ui->console->append(location);
    ui->robotLocationBox->addItem(location);
    //qDebug() << location;
}
