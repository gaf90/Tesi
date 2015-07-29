#include "cameramanager.h"
#include "baseStation/graphicparams.h"
#include <QHash>

namespace BaseStation{

CameraManager::CameraManager(QObject *parent) :
    QObject(parent), selectedRobot(0), focusView(new graphics::CameraQLabel(666)),
    thumbnailView(new QHash<uint, graphics::CameraQLabel*>())
{
    focusCameraWidth = FOCUS_CAMERA_WIDTH;
    focusCameraHeight = FOCUS_CAMERA_HEIGHT;
    thumbnailCameraWidth = THUMBNAIL_VIEW_WIDTH;
    thumbnailCameraHeight = THUMBNAIL_VIEW_HEIGHT;
    QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
//    sizePolicy.setHorizontalStretch(0);
//    sizePolicy.setVerticalStretch(0);
//    sizePolicy.setHeightForWidth(focusView->sizePolicy().hasHeightForWidth());
    focusView->setSizePolicy(sizePolicy);
    focusView->resize(QSize(focusCameraWidth, focusCameraHeight));
//    focusView->setBaseSize(QSize(focusCameraWidth, focusCameraHeight));
    for(int r=0; r<=MAX_ROBOT_ASSUMED; r++){
        contrastValues[r]= 0;
        brightnessValues[r]= 0;
    }
}

void CameraManager::onCameraDataReceived(const uint robotID, const QImage &originalImage)
{
    //No error handling!!! To verify that's free from memory-leaks!
    QImage image = QImage(originalImage);

    if(image.isNull())
        return;

    if(brightnessValues[robotID] != 0){
        changeBrightness(image,brightnessValues[robotID]);
    }

    if(contrastValues[robotID] != 0) {
    changeContrast(image,contrastValues[robotID]);

    }
    QImage image2 = image.scaledToWidth(thumbnailCameraWidth);
    QPixmap pic = QPixmap::fromImage(image2);
    QLabel *l = thumbnailView->value(robotID);
//    QPixmap* p = l->pixmap();
//    delete p;
    if(l != 0)
    {
        QPixmap pic2 = QPixmap::fromImage(image);
        l->setPixmap(pic);

        if(selectedRobot == robotID){
//            QPixmap *pf = focusView->pixmap();
//            delete pf;
            if(focusView->size().width() != FOCUS_CAMERA_WIDTH)
                focusView->setPixmap(pic2.scaledToWidth(focusView->size().width()));
            else
                focusView->setPixmap(pic2);
        }
    }
}

void CameraManager::configureThumbnails(int maxRobots)
{
    for(int robotID = 0; robotID < maxRobots; robotID++)
    {
        graphics::CameraQLabel *label = new graphics::CameraQLabel(robotID);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMinimumSize(QSize(thumbnailCameraWidth, thumbnailCameraHeight));
        label->setBaseSize(QSize(0, 0));
        label->setFrameShape(QFrame::Panel);
        label->setLineWidth(3);
        label->resize(THUMBNAIL_VIEW_WIDTH, THUMBNAIL_VIEW_HEIGHT);
        if(robotID == 0)
            label->setFrameShadow(QFrame::Plain);
        else
            label->setFrameShadow(QFrame::Raised);
        thumbnailView->insert(robotID, label);
        connect(label, SIGNAL(clicked(uint)),this, SLOT(onCameraViewClicked(uint)));
    }
}

void CameraManager::onRobotUnreachable(uint robotID)
{
    QImage image = QImage(":/unreachableCameraImage");
    QImage image2 = image.scaledToWidth(thumbnailCameraWidth);
    QPixmap pic = QPixmap::fromImage(image2);
    QLabel *l = thumbnailView->value(robotID);
    if(l != 0)
    {
        QPixmap pic2 = QPixmap::fromImage(image);
        l->setPixmap(pic);

        if(selectedRobot == robotID){
            focusView->setPixmap(pic2);
        }
    }
}


QLabel * CameraManager::getFocusView()
{
    return this->focusView;
}

QLabel * CameraManager::getThumbnail(uint robotID)
{
    return thumbnailView->value(robotID);
}

void CameraManager::onCameraViewClicked(uint id)
{
    if(id != selectedRobot)
       emit sigSelectedRobotChangedCM(id);
}

void CameraManager::onSelectedRobotChangedCM(uint robotID)
{
    thumbnailView->value(robotID)->setFrameShadow(QFrame::Plain);
    thumbnailView->value(selectedRobot)->setFrameShadow(QFrame::Raised);
    this->selectedRobot = robotID;
}
void CameraManager::changeBrightness(QImage& image, int brightnessValue){


        QColor oldColor;
        int r,g,b;

        for(int x=0; x<image.width(); x++){
            for(int y=0; y<image.height(); y++){
                oldColor = QColor(image.pixel(x,y));
                r = oldColor.red() + brightnessValue;
                g = oldColor.green() + brightnessValue;
                b = oldColor.blue() + brightnessValue;

                //we check if the new values are between 0 and 255
                r = qBound(0, r, 255);
                g = qBound(0, g, 255);
                b = qBound(0, b, 255);
                image.setPixel(x,y, qRgb(r,g,b));
            }

        }

    }
void CameraManager::changeContrast(QImage& image, int contrastValue){
    QColor oldColor;
    int r,g,b, factor;
    factor =(259 * (contrastValue + 255)) / (255 * (259 - contrastValue));

    for(int x=0; x<image.width(); x++){
        for(int y=0; y<image.height(); y++){
            oldColor = QColor(image.pixel(x,y));
            r = factor * (oldColor.red() - 128) + 128;
            g =  factor * (oldColor.green() - 128) + 128;
            b =  factor * (oldColor.blue() - 128) + 128;

            //we check if the new values are between 0 and 255
            r = qBound(0, r, 255);
            g = qBound(0, g, 255);
            b = qBound(0, b, 255);
            image.setPixel(x,y, qRgb(r,g,b));
        }
    }

}
void CameraManager::onBrightnessChanged( int brightnessValue, uint robot){
    brightnessValues[robot]=brightnessValue;
    //qDebug() << "Camera manager cambiato valore " << brightnessValue ;
}
void CameraManager::onContrastChanged( int contrastValue, uint robot){
    contrastValues[robot]= contrastValue;
    //qDebug() << "Camera manager cambiato valore " << contrastValue ;
}
}
