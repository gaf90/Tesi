#ifndef INFOMANAGER_H
#define INFOMANAGER_H

#include <QObject>
#include <QHash>
#include "data/infomessage.h"
#include "graphics/modules/singlerobotinfowidget.h"
#define MAX_ROBOT_ASSUMED 20
namespace BaseStation{
/**
* This class is used to manage every information regarding single robots or groups of robots.
* It exposes two main graphical components that show such info: singleRobotInfo and GroupInfo,
* each of them as QWidget elements.
*/
class InfoManager : public QObject
{
    Q_OBJECT
public:
    explicit InfoManager(QObject *parent = 0, uint nRobots = 0);

    //Getters of the UI graphics components
    graphics::SingleRobotInfoWidget* getSingleRobotInfoWidget();
//    QWidget* getGroupInfoWidget();


signals:
    void sigSendInfoMessage(uint robotID);
    void sigChangeModuleStatusIM(QString module, bool enabled, uint robot);
    void signalBrightnessChanged(int newValue, uint robot);
    void signalContrastChanged(int newValue, uint robot);
    void signalAutoModeOn();
    void signalRobotNearVictim();

public slots:

//-------------------------Handling of the single robot selection---------------------------------//
    /**
    * This slot handles the selection of a new, different robot. It must trigger the request for
    * information about the single robot, or evenctually retrieve it from old data!
    *
    * @param id the uint representing the robot's identification code
    */
    void onSelectedRobotChangedIM(uint id);

//--------------------------Handling of incoming information--------------------------------------//
    /**
    * This slot handles incoming data about robots. This can be stored for future uses even if the
    * robot is not currently selected. Some messages can be dropped, if not useful.
    *
    * @param message an InfoMessage object containing the required information.
    * @param robot the ID number of the robot that sent the message.
    */
    void onNewInfoMessageReceived(Data::InfoMessage message, uint robot);

    void onChangeModuleStatusIM(QString module, bool enabled);

private slots:
    void onModuleActivation(QString module, bool enabled);
    void onBrightnessChanged(int newValue);
    void onContrastChanged(int newValue);
    void onAutoModeOn();
    void onRobotNearVictim();


private:
//--------------------Single robot info
    typedef struct{
        uint robotID;
        double batteryStatus;
        double pitchAngle;
        double rollAngle;
        QHash<uint, double> *wssConnections;
        QHash<QString, bool> *moduleActivations;
        QString mission;
        double signalStrenght;
        //...and others
    } t_singleRobot;


    int brightnessValues[MAX_ROBOT_ASSUMED];
    int contrastValues[MAX_ROBOT_ASSUMED];
    bool robotNearVictim[MAX_ROBOT_ASSUMED];

    //QHash of the known robots. Here the information about already queried robots is stored!
    QHash<uint, t_singleRobot> *knownRobots;

    //Penso all'eventualit�  di implementarmi un widget ad hoc per gestire la cosa...
    graphics::SingleRobotInfoWidget *singleRobotInfoView;

    uint selectedRobot;

    uint nRobots;

//---------Group robot info --> How to handle groups informations??? Bisogna prima definire gli high
         //level commands
//    QHash<QString, *Group> *groups;

//----Utility functions

    void requestInformation();
    void showInformation(bool interacting);

};

}
#endif // INFOMANAGER_H
