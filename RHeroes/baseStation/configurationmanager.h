#ifndef CONFIGURATIONMANAGER_H
#define CONFIGURATIONMANAGER_H

#include <QObject>
#include "graphics/modules/configurationwindow.h"

namespace BaseStation{

class ConfigurationManager : public QObject
{
    Q_OBJECT
public:
    explicit ConfigurationManager(QObject *parent = 0);

signals:
    /**
    * This signal enables/disables a single robot module
    */
    void sigChangeModuleStatusCFM(QString moduleName, bool enabled);
    /**
    * Changes the value of a specific configuration param, to be parsed by the receiver module both
    * in base station and single robot.
    */
    void sigConfigurationChanged(QString param, QString value);

    void signalVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                          double hMax, double sMax, double vMax);

    void signalMessageRelevanceConfiguration(int victimDetection, int semanticMapping,
                                             int feedbackMessage, int faultDetection);

    void signalMinimalPrioritySetted(int value);

public slots:
    //! Shows the configuration GUI, from wich it is possible to configure some param and module
    void onShowModuleActivation(); //Eventualmente parametrico

private slots:
    void onModuleActivationChanged(QString name, bool enabled);

    void onVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                        double hMax, double sMax, double vMax);

    void onMessageRelevanceConfiguration(int victimDetection, int semanticMapping,
                                         int feedbackMessage, int faultDetection);

    void onMinimalPrioritySetted(int value);


private:
    graphics::ConfigurationWindow *gui;

};

}
#endif // CONFIGURATIONMANAGER_H
