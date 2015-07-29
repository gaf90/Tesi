#ifndef CONFIGURATIONWINDOW_H
#define CONFIGURATIONWINDOW_H

#include <QMainWindow>
#include <QSpinBox>
#include <QSlider>

namespace Ui {
    class ConfigurationWindow;
}

namespace graphics{

class ConfigurationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ConfigurationWindow(QWidget *parent = 0);
    ~ConfigurationWindow();

    void closeEvent(QCloseEvent *);

signals:
    /**
    * This signal enables/disables a single robot module
    */
    void sigChangeModuleStatusCFM(QString moduleName, bool enabled);

    /**
    * Changes the value of a specific configuration param, to be parsed by the receiver module both
    * in base station and single robot.
    */
    void signalConfigurationChanged(QString param, QString value);

    void sigVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                            double hMax, double sMax, double vMax);

    void sigMessageRelevanceConfiguration(int victimDetection, int semanticMapping,
                                             int feedbackMessage, int faultDetection);

    void signalMinimalPrioritySetted(int value);

private slots:
//    void onVictimDetectionModuleActivationChanged(int state);
//    void onExplorationModuleActivationChanged(int state);
//    void onSemanticInfoModuleActivationChanged(int state);
    void onConfirmConfigurations();
    void onAbortConfigurations();

    void makeSpinBoxesConsistent();

    void makeSlidersConsistent();

private:
    Ui::ConfigurationWindow *ui;
    bool isVictimDetectionActive, isExplorationActive, isSemanticMappingActive;

    double hMin, sMin, vMin, hMax, sMax, vMax;

    int feedbackRel, victimRel, faultRel, semanticRel;

    QHash<QSpinBox *, QSlider*> *spinBoxes;
    QHash<QSlider*, QSpinBox*> *sliders;

    int minimalPriority;

    void setupConsistentView();

};

}
#endif // CONFIGURATIONWINDOW_H
