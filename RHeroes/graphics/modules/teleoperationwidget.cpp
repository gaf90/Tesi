#include "teleoperationwidget.h"
#include "ui_teleoperationwidget.h"
#include <QKeyEvent>

#define TELE_WIDGET_UNKNOWN_TYPE "unknown"
#define TELE_WIDGET_AERIAL_TYPE "aerial"
#define TELE_WIDGET_TERRESTRIAL_TYPE "terrestrial"

#define TELE_WIDGET_MAX_SPEED 1//1.
#define TELE_WIDGET_ROTATION_SPEED_FACTOR 3.
#define TELE_WIDGET_BACKWARD_SPEED_FACTOR 1.3
#define TELE_WIDGET_KENAF_MAX_SPEED 20
#define TIME_STEP                   500
namespace graphics{

TeleoperationWidget::TeleoperationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TeleoperationWidget),
    robotID(0),
    right(false), forward(false), back(false), left(false), isActive(true),
    robotType(new QHash<uint, QString>()), kenafMaxSpeed(TELE_WIDGET_KENAF_MAX_SPEED)
{
    ui->setupUi(this);
    speedTimer  = new QTimer();
    //speedTimer->setSingleShot(true);

    connect(speedTimer, SIGNAL(timeout()),this,SLOT(onIncreaseSpeed()));
    connect(this, SIGNAL(sigSendTeleoperationCommand(double,double,uint)), this, SLOT(updateScreens(double,double)));

    connect(ui->backButton, SIGNAL(pressed()), this, SLOT(updateInputs()));
    connect(ui->forwardButton, SIGNAL(pressed()), this, SLOT(updateInputs()));
    connect(ui->rightButton, SIGNAL(pressed()), this, SLOT(updateInputs()));
    connect(ui->leftButton, SIGNAL(pressed()), this, SLOT(updateInputs()));

    connect(ui->backButton, SIGNAL(released()), this, SLOT(updateInputs()));
    connect(ui->forwardButton, SIGNAL(released()), this, SLOT(updateInputs()));
    connect(ui->rightButton, SIGNAL(released()), this, SLOT(updateInputs()));
    connect(ui->leftButton, SIGNAL(released()), this, SLOT(updateInputs()));

    connect(ui->toAerialButton, SIGNAL(clicked()), this, SLOT(onSwitchUI()));

    connect(ui->kenafSpeedSlider, SIGNAL(sliderMoved(int)), this, SLOT(updateKenafSpeed(int)));

    kenafList = new QList<uint>();
}

TeleoperationWidget::~TeleoperationWidget()
{
    delete ui;
    if(speedTimer->isActive()){
        speedTimer->stop();
    }
    delete speedTimer;
}

void TeleoperationWidget::handleKeyPressEvent(QKeyEvent *e)
{
    this->keyPressEvent(e);
}

void TeleoperationWidget::handleKeyReleaseEvent(QKeyEvent *e)
{
    this->keyReleaseEvent(e);
}

void TeleoperationWidget::setActive(bool active)
{
    this->isActive = active;
}

void TeleoperationWidget::addKenaf(uint id)
{
    if(!kenafList->contains(id))
        kenafList->append(id);
}

void TeleoperationWidget::onSwitchToGlobal()
{
    this->hide();
}

void TeleoperationWidget::onSwitchToTeleoperation()
{
    this->show();
}

void TeleoperationWidget::onSelectedRobotChangedTW(uint robotID)
{
    if(speedTimer->isActive()){
        speedTimer->stop();
    }
    if(forward || back || right || left)
        emit sigSendTeleoperationCommand(0,0,this->robotID);
    this->robotID = robotID;
    forward = false;
    back = false;
    right = false;
    left = false;
    QString type = robotType->value(robotID, TELE_WIDGET_UNKNOWN_TYPE);
    if(isActive && type == TELE_WIDGET_AERIAL_TYPE){
        this->onSwitchUI();
    }
}

void TeleoperationWidget::onSpawnRobotTW(const QString, const QString, const QString,
                                         const uint nActiveRobot, uint , bool aerial)
{
    if(aerial)
        robotType->insert(nActiveRobot, TELE_WIDGET_AERIAL_TYPE);
    else
        robotType->insert(nActiveRobot, TELE_WIDGET_TERRESTRIAL_TYPE);
}

void TeleoperationWidget::updateKenafSpeed(int speed)
{
    this->kenafMaxSpeed = speed;
}

void TeleoperationWidget::updateInputs()
{
    bool changed = false;
    if(back != ui->backButton->isDown() || forward != ui->forwardButton->isDown() ||
             left != ui->leftButton->isDown() || right != ui->rightButton->isDown())
        changed = true;
    back = ui->backButton->isDown();
    forward = ui->forwardButton->isDown();
    right = ui->rightButton->isDown();
    left = ui->leftButton->isDown();
    if(changed)
        changeWheelSpeed();
}

void TeleoperationWidget::changeWheelSpeed()
{
    double rotationFactor = TELE_WIDGET_ROTATION_SPEED_FACTOR;
    if(kenafList->contains(robotID))
        rotationFactor = 1.;
    double maxSpeed = TELE_WIDGET_MAX_SPEED;
    if(superKenaf)
    {
        //maxSpeed = kenafMaxSpeed * 0.5;
        speedToBeReached = kenafMaxSpeed* 0.5;
        currentSpeed = maxSpeed;
        speedTimer->start(TIME_STEP);
        emit sigSendTeleoperationCommand(maxSpeed, maxSpeed, this->robotID);
        return;
    }
    if(megaKenaf)
    {
        //maxSpeed = kenafMaxSpeed;
        speedToBeReached = kenafMaxSpeed;
        currentSpeed = maxSpeed;
        speedTimer->start(500);
        emit sigSendTeleoperationCommand(maxSpeed, maxSpeed, this->robotID);
        return;
    }

    if(!back && !forward && !left && !right)
        emit sigSendTeleoperationCommand(0,0,this->robotID);
    else
    {
        if(back && forward){
            emit sigSendTeleoperationCommand(0,0,this->robotID);
        }
        else if(left && right){
            emit sigSendTeleoperationCommand(0,0,this->robotID);
        }
        else{                                       //no conflicts
            if(forward){
                if(forward && right)
                    emit sigSendTeleoperationCommand(1,0.5,this->robotID);
                else if(forward && left)
                    emit sigSendTeleoperationCommand(0.5,1,this->robotID);
                else if(!left && !right)
                    emit sigSendTeleoperationCommand(maxSpeed, maxSpeed, this->robotID);
            }
            if(back){
                if(back && right)
                    emit sigSendTeleoperationCommand(-1,-0.5,this->robotID);
                else if(back && left)
                    emit sigSendTeleoperationCommand(-0.5,-1,this->robotID);
                else if(!left && !right)
                    emit sigSendTeleoperationCommand(-maxSpeed / TELE_WIDGET_BACKWARD_SPEED_FACTOR,
                                                  -maxSpeed / TELE_WIDGET_BACKWARD_SPEED_FACTOR, this->robotID);
            }
            if(right && !forward && !back){
                emit sigSendTeleoperationCommand(maxSpeed / rotationFactor ,
                                              -maxSpeed / rotationFactor,
                                              this->robotID);
            }
            if(left && !forward && !back){
                emit sigSendTeleoperationCommand(-maxSpeed / rotationFactor ,
                                              maxSpeed / rotationFactor,
                                              this->robotID);
            }
        }
    }
}

void TeleoperationWidget::onIncreaseSpeed(){
    currentSpeed+=1*(double)TIME_STEP/1000;
    emit sigSendTeleoperationCommand(currentSpeed, currentSpeed, this->robotID);
    if(currentSpeed>=speedToBeReached){
        speedTimer->stop();
    }
}

void TeleoperationWidget::onSwitchUI()
{
    emit signalSwitchRobotType();
}

void TeleoperationWidget::updateScreens(double sLeft, double sRight)
{
    ui->leftSpeedLCD->display(sLeft);
    ui->rightSpeedLCD->display(sRight);
}

void TeleoperationWidget::keyPressEvent(QKeyEvent *e)
{
    if(e->isAutoRepeat() || !isActive){
        return;
    }
    if(speedTimer->isActive()){
        speedTimer->stop();
    }
    if(((e->key() == Qt::Key_W) || (e->key() == Qt::Key_S) || (e->key() == Qt::Key_D) ||
        (e->key() == Qt::Key_A)) || (e->key() == Qt::Key_K) || (e->key() == Qt::Key_E) || (e->key() == Qt::Key_Q))
        e->accept();
    else
        e->ignore();
    forward = (e->key() == Qt::Key_W);
    back = (e->key() == Qt::Key_S);
    right = (e->key() == Qt::Key_D);
    left = (e->key() == Qt::Key_A);
    superKenaf = (e->key() == Qt::Key_Q);
    megaKenaf = (e->key() == Qt::Key_E);
    ui->forwardButton->setDown(e->key() == Qt::Key_W);
    ui->backButton->setDown(e->key() == Qt::Key_S);
    ui->rightButton->setDown(e->key() == Qt::Key_D);
    ui->leftButton->setDown(e->key() == Qt::Key_A);
    changeWheelSpeed();
    if(e->key() == Qt::Key_K)
        this->onSwitchUI();
}

void TeleoperationWidget::keyReleaseEvent(QKeyEvent *e)
{
    if(e->isAutoRepeat() || !isActive)
        return;

    if(speedTimer->isActive()){
        speedTimer->stop();
    }

    if(e->key() == Qt::Key_W)
    {
        forward = false;
        ui->forwardButton->setDown(false);
    }
    if(e->key() == Qt::Key_S)
    {
        back = false;
        ui->backButton->setDown(false);
    }
    if(e->key() == Qt::Key_D)
    {
        right = false;
        ui->rightButton->setDown(false);
    }
    if(e->key() == Qt::Key_A)
    {
        left = false;
        ui->leftButton->setDown(false);
    }
    if((e->key() == Qt::Key_Q))
        superKenaf = false;
    if((e->key() == Qt::Key_E))
        megaKenaf = false;
    changeWheelSpeed();
}

}

