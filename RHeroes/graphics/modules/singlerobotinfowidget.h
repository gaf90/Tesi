#ifndef SINGLEROBOTINFOWIDGET_H
#define SINGLEROBOTINFOWIDGET_H

#include <QWidget>

namespace Ui {
class SingleRobotInfoWidget;
}

namespace graphics{

class SingleRobotInfoWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SingleRobotInfoWidget(QWidget *parent = 0);
    ~SingleRobotInfoWidget();

    void setRobot(QString robotName);
    void setBatteryStatus(int charge);
    void addWirelessInformation(QString robot, double power);
    void clearWirelessTable();
    void setActivationStatus(bool global, bool exploration, bool slam, bool victim);
    void setMission(QString mission);
    void setBrightnessValue(int value);
    void setContrastValue(int value);
    void setSignalStrenghtStatus(int signalStrenght);


signals:
    void sigChangeModuleStatusSIW(const QString &module, bool enable);
    void signalAutoModeOn();
    void signalBrightnessChanged(int newValue);
    void signalContrastChanged(int newValue);
    void signalRobotNearVictim();

private slots:
    void onGlobalSetted(bool state);
    void onSubmitActivations();
    void onSomethingActivated();
    void onBrightnessChanged(int value);
    void onContrastChanged(int value);
    void onAutoModeActivated();
    void onRobotNearVictim();

private:
    Ui::SingleRobotInfoWidget *ui;
    bool changed;

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
};

}
#endif // SINGLEROBOTINFOWIDGET_H
