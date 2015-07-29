#include "showvictimwidget.h"
#include "ui_showvictimwidget.h"
#include <QImage>
#define SCALED_WIDTH 100

namespace graphics{

ShowVictimWidget::ShowVictimWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShowVictimWidget),
    images(),
    actualFrame(0)
{
    ui->setupUi(this);
    connect(ui->nextScrollButton_3, SIGNAL(clicked()), this, SLOT(nextFrame()));
    connect(ui->previousScrollButton_3, SIGNAL(clicked()), this, SLOT(previousFrame()));
}

ShowVictimWidget::~ShowVictimWidget()
{
    delete ui;
}

void ShowVictimWidget::setImages(const QList<QImage> &imgs)
{
    actualFrame = 0;
    images.clear();
    for(int i = 0; i < imgs.size(); i++){
        images.append(imgs.at(i).scaledToWidth(SCALED_WIDTH));
    }
    showFrame();
}

void ShowVictimWidget::nextFrame()
{
    if(actualFrame < images.size() -1)
        actualFrame++;
    else
        actualFrame = 0;
    if(images.size() > 0)
        showFrame();
}

void ShowVictimWidget::previousFrame()
{
    if(actualFrame > 0)
        actualFrame--;
    else
        actualFrame = images.size()-1;
    if(images.size() > 0)
        showFrame();
}

void ShowVictimWidget::showFrame()
{
    if(actualFrame >= 0 && actualFrame < images.size()){
        QPixmap pic = QPixmap::fromImage(images.at(actualFrame));
        ui->imgLabel->setPixmap(pic);
    }
    else
    {
        QPixmap pic = QPixmap::fromImage(QImage(":/noImageAvailable").scaledToWidth(SCALED_WIDTH));
        ui->imgLabel->setPixmap(pic);
    }
}

}
