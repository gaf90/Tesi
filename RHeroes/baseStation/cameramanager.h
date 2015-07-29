#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <QObject>
#include <QLabel>

#include "graphics/utils/cameraqlabel.h"
#define MAX_ROBOT_ASSUMED 20
namespace BaseStation{

/**
* This class handles camera data coming from the robots. It shows such data in 2 formats:
* thumbnails and focused view. The first one combines the camera data of all the robots and shows it
* in a compact way (and at a low framerate). The second one shows the camera stream of the selected
* robotat full size and framerate.
*/
class CameraManager : public QObject
{
    Q_OBJECT
public:
    explicit CameraManager(QObject *parent = 0);

/*-------------------------------------------------------------------------------------------------
                       Methods to retrieve the graphical components
-------------------------------------------------------------------------------------------------*/
    QLabel* getFocusView();
    QLabel* getThumbnail(uint robotID);


signals:

    /**
    * This signal notifies that the operator changed the selected robot.
    * This must be handled by the BaseStationCore module and be forwarded to all the interested
    * modules.
    * @param robotID the id of the new selected robot.
    */
    void sigSelectedRobotChangedCM(uint robotID);

public slots:

    /**
    * Shows the frame on the focusedView and/or on the thumbnail view, depending on the robotID.
    *@param robotID ID of the robot that transmitted the frame
    * @param image the frame to be show.
    */
    void onCameraDataReceived(const uint robotID, const QImage &image);

    /**
    * Handles the change of the selected robot.
    * #param robotID the ID of the new selected robot.
    */
    void onSelectedRobotChangedCM(uint robotID);

    /**
    * This creates the support for a new robot spawned.
    */
    void configureThumbnails(int maxRobots);

    /**
    * This slot handles the unreachability of a robot, showing the operator a proper image.
    */
    void onRobotUnreachable(uint robotID);
    void onBrightnessChanged(int brightnessValue,uint robot);
    void onContrastChanged(int brightnessValue,uint robot);

private slots:
/*-------------------------------------------------------------------------------------------------
                       Methods to allow the change of selected robot
                                    in this component
-------------------------------------------------------------------------------------------------*/
    void onCameraViewClicked(uint id);
    void changeBrightness(QImage& image, int brightnessValue);
    void changeContrast(QImage& image, int contrastValue);

private:
    uint selectedRobot;
    graphics::CameraQLabel *focusView;
    QHash <uint, graphics::CameraQLabel*> *thumbnailView;

    int focusCameraWidth;
    int focusCameraHeight;
    int thumbnailCameraWidth;
    int thumbnailCameraHeight;
    int brightnessValues[MAX_ROBOT_ASSUMED];
    int contrastValues[MAX_ROBOT_ASSUMED];
};


}
#endif // CAMERAMANAGER_H
