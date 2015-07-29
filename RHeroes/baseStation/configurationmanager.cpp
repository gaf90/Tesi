#include "configurationmanager.h"

namespace BaseStation{

ConfigurationManager::ConfigurationManager(QObject *parent) :
    QObject(parent), gui(new graphics::ConfigurationWindow())
{
    connect(gui, SIGNAL(sigChangeModuleStatusCFM(QString,bool)),
            this, SLOT(onModuleActivationChanged(QString,bool)));
    connect(gui, SIGNAL(sigVictimDetectionConfiguration(double,double,double,double,double,double)),
            this, SLOT(onVictimDetectionConfiguration(double,double,double,double,double,double)));
    connect(gui, SIGNAL(sigMessageRelevanceConfiguration(int,int,int,int)),
            this, SLOT(onMessageRelevanceConfiguration(int,int,int,int)));
    connect(gui, SIGNAL(signalMinimalPrioritySetted(int)), this, SLOT(onMinimalPrioritySetted(int)));
}

void ConfigurationManager::onShowModuleActivation()
{
    gui->showNormal();
}

void ConfigurationManager::onModuleActivationChanged(QString name, bool enabled)
{
    emit sigChangeModuleStatusCFM(name, enabled);
}

void ConfigurationManager::onVictimDetectionConfiguration(double hMin, double sMin, double vMin,
                                                          double hMax, double sMax, double vMax)
{
    emit signalVictimDetectionConfiguration(hMin, sMin, vMin, hMax, sMax, vMax);
}

void ConfigurationManager::onMessageRelevanceConfiguration(int victimDetection, int semanticMapping,
                                                           int feedbackMessage, int faultDetection)
{
    emit signalMessageRelevanceConfiguration(victimDetection, semanticMapping, feedbackMessage, faultDetection);
}

void ConfigurationManager::onMinimalPrioritySetted(int value)
{
    emit signalMinimalPrioritySetted(value);
}

}
