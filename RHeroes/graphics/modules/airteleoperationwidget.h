#ifndef AIRTELEOPERATIONWIDGET_H
#define AIRTELEOPERATIONWIDGET_H

#include <QWidget>

namespace Ui {
class AirTeleoperationWidget;
}

namespace graphics{

class AirTeleoperationWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit AirTeleoperationWidget(QWidget *parent = 0);
    ~AirTeleoperationWidget();

    void handleKeyPressEvent(QKeyEvent *e);
    void handleKeyReleaseEvent(QKeyEvent *e);

    void setActive(bool active);

signals:
    void sigSendAirTeleoperationCommand(double altitude, double linear, double lateral,
                                double rotational, uint robotID);

    void signalSwitchRobotType();

public slots:
    /**
    * slot used to manage the selected robot change .
    */
    void onSelectedRobotChangedTW(uint robotID);

    void onSpawnRobotTW(const QString, const QString,
                        const QString, const uint nActiveRobot, uint, bool aerial);

private slots:
    void onActionPerformed();
    void onSwitchUI();
    void onSpeedChanged(int speed);

private:
    Ui::AirTeleoperationWidget *ui;
    uint robotID;
    int speedFactor;
    bool isActive;

    QHash<uint, QString> *robotType;

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
};

}
#endif // AIRTELEOPERATIONWIDGET_H
