#include "robotui.h"
#include "ui_robotui.h"
#include "shared/constants.h"
#include "data/usarmessage.h"
#include "data/wheelmessage.h"
#include <QDebug>

using namespace Data;

RobotUi::RobotUi(uint robotId, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RobotUi), robotId(robotId)
{
    ui->setupUi(this);
    connect(ui->actionButton, SIGNAL(clicked()), this, SLOT(onSendActionButtonClicked()));
    connect(ui->disconnectButton, SIGNAL(clicked()), this, SLOT(onDisconnectButtonClicked()));
    connect(ui->rawButton, SIGNAL(clicked()), this, SLOT(onSendRawButtonClicked()));
    connect(ui->flipButton, SIGNAL(clicked()), this, SLOT(onFlipButtonClicked()));
    connect(ui->moveForwardButton, SIGNAL(clicked()), this, SLOT(onGoForwardButtonClicked()));
    connect(ui->moveBackwardtButton, SIGNAL(clicked()), this, SLOT(onGoBackwardButtonClicked()));
    connect(ui->moveLeftButton, SIGNAL(clicked()), this, SLOT(onTurnLeftButtonClicked()));
    connect(ui->moveRightButton, SIGNAL(clicked()), this, SLOT(onTurnRightButtonClicked()));
    connect(ui->rotateLeftButton, SIGNAL(clicked()), this, SLOT(onRotateLeftClicked()));
    connect(ui->rotateRightButton, SIGNAL(clicked()), this, SLOT(onRotateRightClicked()));
    connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
    //connect(ui->connectToRobotButton, SIGNAL(clicked()), this, SLOT(onConnectToRobotButtonClicked()));

}

RobotUi::~RobotUi()
{
    delete ui;
}

void RobotUi::onSendRawButtonClicked()
{
    //Collect data from UI and create a message for USAR
    QString rawCommand = ui->rawCmdLine->text();
    USARMessage *message = new USARMessage(rawCommand);
    //Emit the message for usar
    emit emitMessage(*message, robotId);

    //When the emit returns, all the connected slots have been executed.
    //I can safely delete the message
    delete message;

}

void RobotUi::onSendActionButtonClicked()
{
    //Collect data from UI.
//    double initRot = ui->initRotSpin->value();
//    double transl = ui->traslSpin->value();
//    double finRot = ui->endRotSpin->value();

    //Build a BuddyMessage that the BaseStation will
    //Forward to the right robot.
}

void RobotUi::onDisconnectButtonClicked()
{
    emit signalDisconnect();
}

void RobotUi::onGoForwardButtonClicked()
{
    WheelMessage message(0.5, 0.5);
    emit emitMessage(message, robotId);

}

void RobotUi::onGoBackwardButtonClicked()
{
    WheelMessage message(-0.5, -0.5);
    emit emitMessage(message, robotId);
}

void RobotUi::onTurnLeftButtonClicked()
{
    WheelMessage message(0, 0.5);
    emit emitMessage(message, robotId);
}

void RobotUi::onTurnRightButtonClicked()
{
    WheelMessage message(0.5, 0);
    emit emitMessage(message, robotId);
}

void RobotUi::onFlipButtonClicked()
{

}

void RobotUi::onRotateLeftClicked()
{
    WheelMessage message(-0.5, 0.5);
    emit emitMessage(message, robotId);
}

void RobotUi::onRotateRightClicked()
{
    WheelMessage message(0.5, -0.5);
    emit emitMessage(message, robotId);
}

void RobotUi::onStopButtonClicked()
{
    WheelMessage message(0, 0);
    emit emitMessage(message, robotId);
}

void RobotUi::onConnectToRobotButtonClicked()
{

}

void RobotUi::showError(const QString &error)
{
    qDebug() << error;
    //ui->console->append(error);
}

void RobotUi::onImage(const QImage &image)
{

    QPixmap pic = QPixmap::fromImage(image);
    ui->pictureLabel->setPixmap(pic);

}

uint RobotUi::getRobotId(){
    return robotId;
}
