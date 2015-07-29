#include "singlerobotinfowidget.h"
#include "ui_singlerobotinfowidget.h"
#include "shared/constants.h"
#include <QKeyEvent>

namespace graphics{

SingleRobotInfoWidget::SingleRobotInfoWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SingleRobotInfoWidget)
{
    ui->setupUi(this);
    ui->connectionTable->setSortingEnabled(false);
    connect(ui->globalStopCheckBox, SIGNAL(clicked(bool)), this, SLOT(onGlobalSetted(bool)));
    connect(ui->submitButton, SIGNAL(clicked()), this, SLOT(onSubmitActivations()));
    connect(ui->explorationStopCheckBox, SIGNAL(clicked()), this, SLOT(onSomethingActivated()));
    connect(ui->victimDetectionStopCheckBox, SIGNAL(clicked()), this, SLOT(onSomethingActivated()));
    connect(ui->slamStopCheckBox, SIGNAL(clicked()), this, SLOT(onSomethingActivated()));
    connect(ui->slamCheckBox, SIGNAL(clicked()), this, SLOT(onSomethingActivated()));
    connect(ui->BrightnessBar,SIGNAL(valueChanged(int)),this,SLOT(onBrightnessChanged(int)));
    connect(ui->ContrastBar,SIGNAL(valueChanged(int)),this,SLOT(onContrastChanged(int)));
    connect(ui->autoModeButton,SIGNAL(clicked()),this, SLOT(onAutoModeActivated()));
    connect(ui->victimFoundButton,SIGNAL(clicked()),this, SLOT(onRobotNearVictim()));
    changed = false;
}

SingleRobotInfoWidget::~SingleRobotInfoWidget()
{
    delete ui;
}

void SingleRobotInfoWidget::setRobot(QString robotName)
{
    ui->robotNameLabel->setText(robotName);
    changed = false;
}

void SingleRobotInfoWidget::setBatteryStatus(int charge)
{
    ui->batteryStatusBar->setValue(charge);
}

void SingleRobotInfoWidget::setSignalStrenghtStatus(int signalStrenght){
    ui->signalStrenghtBar->setValue(signalStrenght);
}

void SingleRobotInfoWidget::setBrightnessValue(int value)
{

    ui->BrightnessBar->setValue(value);
}
void SingleRobotInfoWidget::setContrastValue(int value)
{

    ui->ContrastBar->setValue(value);
}

void SingleRobotInfoWidget::clearWirelessTable()
{
    ui->connectionTable->clear();
    ui->connectionTable->setRowCount(0);
}

void SingleRobotInfoWidget::setActivationStatus(bool global, bool exploration, bool slam, bool victim)
{
    ui->globalStopCheckBox->setChecked(global);
    ui->explorationStopCheckBox->setChecked(exploration);
    ui->slamStopCheckBox->setChecked(slam);
    ui->victimDetectionStopCheckBox->setChecked(victim);
}

void SingleRobotInfoWidget::setMission(QString mission)
{
    ui->missionLabel->setText(mission);
}

void SingleRobotInfoWidget::addWirelessInformation(QString robot, double power)
{
    int rows = ui->connectionTable->rowCount() + 1;
    ui->connectionTable->setRowCount(rows);
    ui->connectionTable->setItem(rows-1,0, new QTableWidgetItem(robot));
    ui->connectionTable->setItem(rows-1,1, new QTableWidgetItem(QString::number(power)));
}

void SingleRobotInfoWidget::onSubmitActivations()
{
    if(changed)
    {
        emit sigChangeModuleStatusSIW(EXPLORATION, ui->explorationStopCheckBox->isChecked());
        emit sigChangeModuleStatusSIW(VICTIM_DETECTION, ui->victimDetectionStopCheckBox->isChecked());
        emit sigChangeModuleStatusSIW(SEMANTIC_MAPPING, ui->slamStopCheckBox->isChecked());
        emit sigChangeModuleStatusSIW(POARET_SLAM_MODULE, ui->slamCheckBox->isChecked());
    }
    changed = false;
}

void SingleRobotInfoWidget::onSomethingActivated()
{
    changed = true;
    if(ui->explorationStopCheckBox->isChecked() || ui->victimDetectionStopCheckBox->isChecked() ||
            ui->slamStopCheckBox->isChecked())
        ui->globalStopCheckBox->setChecked(false);
}
void SingleRobotInfoWidget::onRobotNearVictim(){
    ui->globalStopCheckBox->setChecked(true);
    onGlobalSetted(true);
    onSubmitActivations();
    emit signalRobotNearVictim();

}

void SingleRobotInfoWidget::keyPressEvent(QKeyEvent *e)
{
    e->ignore();
}

void SingleRobotInfoWidget::keyReleaseEvent(QKeyEvent *e)
{
    e->ignore();
}

void SingleRobotInfoWidget::onGlobalSetted(bool state)
{
    ui->explorationStopCheckBox->setChecked(!state);
    ui->victimDetectionStopCheckBox->setChecked(!state);
    ui->slamStopCheckBox->setChecked(!state);
    changed = true;
}
void SingleRobotInfoWidget::onBrightnessChanged( int value)
{

    QString robot=ui->robotNameLabel->text();
    int id=robot.mid(6).toInt();
    emit signalBrightnessChanged(value);
}

void SingleRobotInfoWidget::onContrastChanged( int value)
{

    QString robot=ui->robotNameLabel->text();
    int id=robot.mid(6).toInt();
    emit signalContrastChanged(value);
}
void SingleRobotInfoWidget::onAutoModeActivated(){
    emit signalAutoModeOn();
}
}
