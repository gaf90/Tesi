#include "showvictimdialog.h"
#include "ui_showvictimdialog.h"
#include "baseStation/graphicparams.h"

namespace graphics{

ShowVictimDialog::ShowVictimDialog(QWidget *parent, uint id, QList<QImage> imgs) :
    QDialog(parent),
    ui(new Ui::ShowVictimDialog),
    victimID(id),
    images(imgs),
    actualFrame(0)
{
    ui->setupUi(this);
    ui->imgLabel->resize(FOCUS_CAMERA_WIDTH, FOCUS_CAMERA_HEIGHT);
    this->setWindowTitle("Victim with id " + QString::number(id) + "maybe found");

    //Connections
    connect(ui->nextScrollButton, SIGNAL(clicked()), this, SLOT(nextFrame()));
    connect(ui->previousScrollButton, SIGNAL(clicked()), this, SLOT(previousFrame()));
    connect(ui->confirmButton, SIGNAL(clicked()), this, SLOT(onVictimAccepted()));
    connect(ui->notAVictimButton, SIGNAL(clicked()), this, SLOT(onVictimRefused()));
    connect(ui->postponeButton, SIGNAL(clicked()), this, SLOT(onPostponeDecision()));
    if(images.size() > 0)
        showFrame();
}

ShowVictimDialog::~ShowVictimDialog()
{
    delete ui;
}

uint ShowVictimDialog::getVictimID()
{
    return this->victimID;
}

void ShowVictimDialog::nextFrame()
{
    if(actualFrame < images.size() -1)
        actualFrame++;
    else
        actualFrame = 0;
    if(images.size() > 0)
        showFrame();
}

void ShowVictimDialog::previousFrame()
{
    if(actualFrame > 0)
        actualFrame--;
    else
        actualFrame = images.size()-1;
    if(images.size() > 0)
        showFrame();
}

void ShowVictimDialog::onVictimAccepted()
{
    emit signalVictimConfirmed(victimID);
}

void ShowVictimDialog::onVictimRefused()
{
    emit signalNoVictimRetrieved(victimID);
}

void ShowVictimDialog::onPostponeDecision()
{
    this->reject();
}

void ShowVictimDialog::showFrame()
{
    QPixmap pic = QPixmap::fromImage(images.at(actualFrame));
    ui->imgLabel->setPixmap(pic);
}

}
