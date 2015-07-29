#ifndef TELEOPERATIONWIDGET_H
#define TELEOPERATIONWIDGET_H

#include <QWidget>
#include <QShortcut>
#include <QTimer>

namespace Ui {
    class TeleoperationWidget;
}


namespace graphics{

class TeleoperationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TeleoperationWidget(QWidget *parent = 0);
    ~TeleoperationWidget();

    void handleKeyPressEvent(QKeyEvent *e);
    void handleKeyReleaseEvent(QKeyEvent *e);

    void setActive(bool active);

    void addKenaf(uint id);

signals:

    void sigSendTeleoperationCommand(double sLeft, double sRight, uint RobotID);

    void signalSwitchRobotType();

public slots:

    /**
    * This slot handles the switching from the teleoperation view to the global one
    */
    void onSwitchToGlobal();

    /**
    * This slot handles the switching from the global view to the teleoperation one
    */
    void onSwitchToTeleoperation();

    /**
    * slot used to manage the selected robot change .
    */
    void onSelectedRobotChangedTW(uint robotID);

    void onSpawnRobotTW(const QString, const QString,
                        const QString, const uint nActiveRobot, uint, bool aerial);

private slots:

    void updateKenafSpeed(int speed);

private:
    Ui::TeleoperationWidget *ui;
    uint robotID;
    bool right, forward, back, left, superKenaf, megaKenaf;

    QShortcut *b, *f, *r, *l;
    bool isActive;

    QHash<uint, QString> *robotType;

    QList<uint> *kenafList;

    QTimer* speedTimer;

    double kenafMaxSpeed;

    double currentSpeed;
    double speedToBeReached;

    void changeWheelSpeed();

private slots:
    void onSwitchUI();

////  Shortcut coarse management
//    void setBack();
//    void setForward();
//    void setLeft();
//    void setRight();

    void updateInputs();

    void updateScreens(double sLeft, double sRight);

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

    void onIncreaseSpeed();
};


}
#endif // TELEOPERATIONWIDGET_H
