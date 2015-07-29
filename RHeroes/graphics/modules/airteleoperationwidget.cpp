#include "airteleoperationwidget.h"
#include "ui_airteleoperationwidget.h"
#include <QKeyEvent>

#define TELE_WIDGET_UNKNOWN_TYPE "unknown"
#define TELE_WIDGET_AERIAL_TYPE "aerial"
#define TELE_WIDGET_TERRESTRIAL_TYPE "terrestrial"

namespace graphics{

AirTeleoperationWidget::AirTeleoperationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AirTeleoperationWidget), robotID(0), speedFactor(1.0), isActive(false),
    robotType(new QHash<uint, QString>())
{
    ui->setupUi(this);
    connect(ui->backwardButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->forwardButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->upButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->downButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->shiftLeftButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->shiftRightButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->rotateLeftButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));
    connect(ui->rotateRightButton, SIGNAL(pressed()), this, SLOT(onActionPerformed()));

    connect(ui->backwardButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->forwardButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->upButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->downButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->shiftLeftButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->shiftRightButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->rotateLeftButton, SIGNAL(released()), this, SLOT(onActionPerformed()));
    connect(ui->rotateRightButton, SIGNAL(released()), this, SLOT(onActionPerformed()));

    connect(ui->toTerrestrialButton, SIGNAL(clicked()),this,SLOT(onSwitchUI()));
    connect(ui->speedSlider, SIGNAL(valueChanged(int)), this, SLOT(onSpeedChanged(int)));
}

AirTeleoperationWidget::~AirTeleoperationWidget()
{
    delete ui;
}

void AirTeleoperationWidget::handleKeyPressEvent(QKeyEvent *e)
{
    this->keyPressEvent(e);
}

void AirTeleoperationWidget::handleKeyReleaseEvent(QKeyEvent *e)
{
    this->keyReleaseEvent(e);
}

void AirTeleoperationWidget::setActive(bool active)
{
    this->isActive = active;
}

void AirTeleoperationWidget::onActionPerformed()
{
    double alt = speedFactor * 0.8 * (ui->upButton->isDown() - ui->downButton->isDown()),
           lin = speedFactor * 0.7 * (ui->forwardButton->isDown() - ui->backwardButton->isDown()),
           lat = speedFactor * 0.7 * (ui->shiftRightButton->isDown() - ui->shiftLeftButton->isDown()),
           rot = speedFactor * 35.0 * (ui->rotateRightButton->isDown() - ui->rotateLeftButton->isDown());
    emit sigSendAirTeleoperationCommand(alt, lin, lat, rot, robotID);
}

void AirTeleoperationWidget::onSwitchUI()
{
    emit signalSwitchRobotType();
}

void AirTeleoperationWidget::onSpeedChanged(int speed)
{
    this->speedFactor = speed;
}

void AirTeleoperationWidget::keyPressEvent(QKeyEvent *e)
{
    if(!isActive)
        return;
    bool catched = false;
    if((e->key() == Qt::Key_Plus && (speedFactor < ui->speedSlider->maximum())) ||
            (((e->key() == Qt::Key_Minus) && (speedFactor > 1))))
    {
        catched = true;
        speedFactor = speedFactor + 1 * ((e->key() == Qt::Key_Plus) - (e->key() == Qt::Key_Minus));
        ui->speedSlider->setValue(speedFactor);
    }
    if(e->isAutoRepeat())
        return;
    if(e->key() == Qt::Key_L)
    {
        catched = true;
        this->onSwitchUI();
    }
    if(e->key() == Qt::Key_A || e->key() == Qt::Key_S || e->key() == Qt::Key_D ||
            e->key() == Qt::Key_Q || e->key() == Qt::Key_W || e->key() == Qt::Key_E ||
            e->key() == Qt::Key_C || e->key() == Qt::Key_Alt)
    {
        catched = true;
        double alt = speedFactor * 0.8 * ((e->key() == Qt::Key_C) - (e->key() == Qt::Key_Alt)),
                lin = speedFactor * 0.7 * ((e->key() == Qt::Key_W) - (e->key() == Qt::Key_S)),
                lat = speedFactor * 0.7 * ((e->key() == Qt::Key_E) - (e->key() == Qt::Key_Q)),
                rot = speedFactor * 35.0 * ((e->key() == Qt::Key_D) - (e->key() == Qt::Key_A));
        // ------------- Graphical feedback
        ui->forwardButton->setDown((e->key() == Qt::Key_W)); ui->backwardButton->setDown((e->key() == Qt::Key_S));
        ui->rotateRightButton->setDown((e->key() == Qt::Key_D)); ui->rotateLeftButton->setDown((e->key() == Qt::Key_A));
        ui->shiftRightButton->setDown((e->key() == Qt::Key_E)); ui->shiftLeftButton->setDown((e->key() == Qt::Key_Q));
        ui->upButton->setDown((e->key() == Qt::Key_C)); ui->downButton->setDown((e->key() == Qt::Key_Alt));

        emit sigSendAirTeleoperationCommand(alt, lin, lat, rot, robotID);
    }
    if(catched)
        e->accept();
    else
        e->ignore();
}

void AirTeleoperationWidget::keyReleaseEvent(QKeyEvent *e)
{
    if(e->isAutoRepeat() || !isActive)
        return;
    double lin = 0, lat = 0, rot = 0 , alt = 0;
    // ------------- Graphical feedback
    if(e->key() == Qt::Key_C)
        ui->upButton->setDown(false);
    if(e->key() == Qt::Key_Alt)
        ui->downButton->setDown(false);
    if(e->key() == Qt::Key_W)
        ui->forwardButton->setDown(false);
    if(e->key() == Qt::Key_S)
        ui->backwardButton->setDown(false);
    if(e->key() == Qt::Key_A)
        ui->rotateLeftButton->setDown(false);
    if(e->key() == Qt::Key_D)
        ui->rotateRightButton->setDown(false);
    if(e->key() == Qt::Key_Q)
        ui->shiftLeftButton->setDown(false);
    if(e->key() == Qt::Key_E)
        ui->shiftRightButton->setDown(false);

    emit sigSendAirTeleoperationCommand(alt, lin, lat, rot, robotID);
    e->accept();
}

void AirTeleoperationWidget::onSelectedRobotChangedTW(uint robotID)
{
    this->robotID = robotID;
    QString type = robotType->value(robotID, TELE_WIDGET_UNKNOWN_TYPE);
    if(isActive && type == TELE_WIDGET_TERRESTRIAL_TYPE){
        this->onSwitchUI();
    }
}

void AirTeleoperationWidget::onSpawnRobotTW(const QString, const QString, const QString,
                                            const uint nActiveRobot, uint, bool aerial)
{
    if(aerial)
        robotType->insert(nActiveRobot, TELE_WIDGET_AERIAL_TYPE);
    else
        robotType->insert(nActiveRobot, TELE_WIDGET_TERRESTRIAL_TYPE);
}

}
