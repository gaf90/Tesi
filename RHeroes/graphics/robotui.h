#ifndef ROBOTUI_H
#define ROBOTUI_H

#include <QMainWindow>
#include <QImage>
#include "data/message.h"

namespace Ui {
    class RobotUi;
}

/**
 * This window represents the user interface of
 * a single robot. From this interface it is
 * possible to see what the robot perceives and
 * to send explicit commands to it.
 */
class RobotUi : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * Constructor for the RobotUi.
     */
    explicit RobotUi(uint robotId, QWidget *parent = 0);
    /**
     * Destructor for the RobotUi.
     */
    virtual ~RobotUi();

    uint getRobotId();

    /**
     * Slot invoked to show the image captured by the robot.
     * @param image the captured image.
     */
    void onImage(const QImage &image);

signals:
    /**
     * Signal emitted when the RobotUI must communicate
     * with other component.
     */
    void emitMessage(const Data::Message &message, uint robotId);
    /**
     * Signal emitted to inform the robot that it has to disconnect
     * from the servers.
     */
    void signalDisconnect();

private slots:
    /**
     * Slot invoked when the "Send Raw" button is pressed.
     * It sends a message to USARSim
     */
    void onSendRawButtonClicked();
    /**
     * Slot invoked when the "Send Action" button is pressed.
     * It sends the rotate-move-rotate action to the robot.
     */
    void onSendActionButtonClicked();
    /**
     * Slot invoked when the "Disconnect" button is pressed.
     * It disconnect the robot, close the window and destroy it.
     */
    void onDisconnectButtonClicked();
    /**
     * Slot invoked when the "^" button is pressed.
     * It commands to the robot to go straight on.
     */
    void onGoForwardButtonClicked();
    /**
     * Slot invoked when the "v" button is pressed.
     * It commands to the robot to go backward.
     */
    void onGoBackwardButtonClicked();
    /**
     * Slot invoked when the "<-" button is pressed.
     * It command the robot to start turning left.
     */
    void onTurnLeftButtonClicked();
    /**
     * Slot invoked when the "->" button is pressed.
     * It command the robot to start turning right.
     */
    void onTurnRightButtonClicked();
    /**
     * Slot invoked when the "Flip" button is pressed.
     * It should restart the robot position.
     */
    void onFlipButtonClicked();
    /**
     * Slot invoked when the "Rotate Left" button is pressed.
     * It stops the robot motion and makes it rotates left on
     * its place.
     */
    void onRotateLeftClicked();
    /**
     * Slot invoked when the "Rotate Right" button is pressed.
     * It stops the robot motion and makes it rotates right on
     * its place.
     */
    void onRotateRightClicked();
    /**
     * Slot invoked when the "Stop" button is pressed.
     * It stops the robot motion.
     */
    void onStopButtonClicked();
    /**
     * Slot invoked when the "Connect to Robot" button is pressed.
     * It allows the robot to communicate with other close robots.
     */
    void onConnectToRobotButtonClicked();

    /**
     * Slot invoked when an error from the servers is received.
     * @param error the error received.
     */
    void showError(const QString &error);



private:
    Ui::RobotUi *ui;
    uint robotId;

};

#endif // ROBOTUI_H
