#include "configurationwindow.h"
#include "ui_configurationwindow.h"
#include "shared/constants.h"
#include "shared/config.h"
#include "baseStation/params.h"

namespace graphics{

ConfigurationWindow::ConfigurationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ConfigurationWindow),
    isVictimDetectionActive(true), isExplorationActive(false), isSemanticMappingActive(true),
    spinBoxes(new QHash<QSpinBox*, QSlider*>()), sliders(new QHash<QSlider*, QSpinBox*>()),
    minimalPriority(0)
{
    ui->setupUi(this);
    hide();
    connect(ui->cancelPushButton, SIGNAL(clicked()), this, SLOT(onAbortConfigurations()));
    connect(ui->okPushButton, SIGNAL(clicked()), this, SLOT(onConfirmConfigurations()));


    /*-------------------------------------------------------------------------------------
                                Initialization Module activations
    -------------------------------------------------------------------------------------*/
    ui->semanticCheckBox->setChecked(isSemanticMappingActive);
    ui->victimDetectionCheckBox->setChecked(isVictimDetectionActive);
    ui->navigationCheckBox->setChecked(isExplorationActive);

    /*-------------------------------------------------------------------------------------
                                Initialization HSV values
    -------------------------------------------------------------------------------------*/
    ui->hMinSpinBox->setValue(Config::HMin);
    ui->sMinSpinBox->setValue(Config::SMin);
    ui->vMinSpinBox->setValue(Config::VMin);
    ui->hMaxSpinBox->setValue(Config::HMax);
    ui->sMaxSpinBox->setValue(Config::SMax);
    ui->vMaxSpinBox->setValue(Config::VMax);

    hMin = ui->hMinSpinBox->value();
    sMin = ui->sMinSpinBox->value();
    vMin = ui->vMinSpinBox->value();
    hMax = ui->hMaxSpinBox->value();
    sMax = ui->sMaxSpinBox->value();
    vMax = ui->vMaxSpinBox->value();

    /*-------------------------------------------------------------------------------------
                             Initialization Messages' relevances
    -------------------------------------------------------------------------------------*/
    QString m = POSSIBLE_MESSAGES;
    QStringList mexTypes = m.split(",");
    QStringList values;
    for(int i = 0; i< mexTypes.size(); i++){
        values = mexTypes.at(i).split("-");
        int relevance = values.at(1).toInt();
        if(values.at(0) == VICTIM_TYPE)
        {
            ui->victimDetectionSlider->setValue(relevance);
            victimRel = relevance;
            ui->victimDetectionSpinBox->setValue(relevance);

            spinBoxes->insert(ui->victimDetectionSpinBox, ui->victimDetectionSlider);
            sliders->insert(ui->victimDetectionSlider, ui->victimDetectionSpinBox);
        }
        if(values.at(0) == SEMANTIC_TYPE)
        {
            semanticRel = relevance;
            ui->semanticMappingSlider->setValue(relevance);
            ui->semanticMappingSpinBox->setValue(relevance);

            spinBoxes->insert(ui->semanticMappingSpinBox, ui->semanticMappingSlider);
            sliders->insert(ui->semanticMappingSlider, ui->semanticMappingSpinBox);
        }
        if(values.at(0) == FAULT_TYPE)
        {
            faultRel = relevance;
            ui->faultDetectionSlider->setValue(relevance);
            ui->faultSpinBox->setValue(relevance);

            spinBoxes->insert(ui->faultSpinBox, ui->faultDetectionSlider);
            sliders->insert(ui->faultDetectionSlider, ui->faultSpinBox);
        }
        if(values.at(0) == FEEDBACK_TYPE)
        {
            feedbackRel = relevance;
            ui->feedbackMessageSlider->setValue(relevance);
            ui->feedbackSpinBox->setValue(relevance);

            spinBoxes->insert(ui->feedbackSpinBox, ui->feedbackMessageSlider);
            sliders->insert(ui->feedbackMessageSlider, ui->feedbackSpinBox);
        }
    }

    spinBoxes->insert(ui->minimalPrioritySpinBox, ui->minimalPrioritySlider);
    sliders->insert(ui->minimalPrioritySlider, ui->minimalPrioritySpinBox);

    ui->minimalPrioritySlider->setValue(minimalPriority);
    ui->minimalPrioritySpinBox->setValue(minimalPriority);

    setupConsistentView();
}

ConfigurationWindow::~ConfigurationWindow()
{
    delete ui;
}

/*--------------------------------------------------------------------------------------------------
                                      Configuration ends
--------------------------------------------------------------------------------------------------*/
void ConfigurationWindow::onConfirmConfigurations()
{
    if((ui->victimDetectionCheckBox->isChecked() && !isVictimDetectionActive) ||
        (!ui->victimDetectionCheckBox->isChecked() && isVictimDetectionActive)    ){
        this->isVictimDetectionActive = ui->victimDetectionCheckBox->isChecked();
        emit sigChangeModuleStatusCFM(VICTIM_DETECTION, isVictimDetectionActive);
    }
    if((ui->navigationCheckBox->isChecked() && !isExplorationActive) ||
        (!ui->navigationCheckBox->isChecked() && isExplorationActive)    ){
        this->isExplorationActive = ui->navigationCheckBox->isChecked();
        emit sigChangeModuleStatusCFM(EXPLORATION, isExplorationActive);
    }
    if((ui->semanticCheckBox->isChecked() && !isSemanticMappingActive) ||
        (!ui->semanticCheckBox->isChecked() && isSemanticMappingActive)    ){
        this->isSemanticMappingActive= ui->semanticCheckBox->isChecked();
        emit sigChangeModuleStatusCFM(SEMANTIC_MAPPING, isSemanticMappingActive);
    }
    if(hMin != ui->hMinSpinBox->value() || sMin != ui->sMinSpinBox->value() ||
       vMin != ui->vMinSpinBox->value() || hMax != ui->hMaxSpinBox->value() ||
       sMax != ui->sMaxSpinBox->value() || vMax != ui->vMaxSpinBox->value())
    {
        hMin = ui->hMinSpinBox->value();
        sMin = ui->sMinSpinBox->value();
        vMin = ui->vMinSpinBox->value();
        hMax = ui->hMaxSpinBox->value();
        sMax = ui->sMaxSpinBox->value();
        vMax = ui->vMaxSpinBox->value();
        emit sigVictimDetectionConfiguration(hMin, sMin, vMin, hMax, sMax, vMax);
    }
    if(victimRel != ui->victimDetectionSlider->value() || semanticRel != ui->semanticMappingSlider->value() ||
         faultRel != ui->faultDetectionSlider->value() || feedbackRel != ui->feedbackMessageSlider->value())
    {
        victimRel = ui->victimDetectionSlider->value();
        semanticRel = ui->semanticMappingSlider->value();
        faultRel = ui->faultDetectionSlider->value();
        feedbackRel = ui->feedbackMessageSlider->value();
        emit sigMessageRelevanceConfiguration(victimRel, semanticRel, feedbackRel, faultRel);
    }
    if(minimalPriority != ui->minimalPrioritySlider->value())
    {
        minimalPriority = ui->minimalPrioritySlider->value();
        emit signalMinimalPrioritySetted(minimalPriority);
    }
    //TODO other configurations

    this->hide();
}

void ConfigurationWindow::onAbortConfigurations()
{
    this->hide();
}

void ConfigurationWindow::makeSpinBoxesConsistent()
{
    foreach(QSlider* slider, sliders->keys())
    {
        sliders->value(slider)->setValue(slider->value());
    }
}

void ConfigurationWindow::makeSlidersConsistent()
{
    foreach(QSpinBox* spin, spinBoxes->keys())
    {
        spinBoxes->value(spin)->setValue(spin->value());
    }
}

void ConfigurationWindow::setupConsistentView()
{
    foreach(QSpinBox* spin, spinBoxes->keys())
    {
        connect(spin, SIGNAL(valueChanged(int)), this, SLOT(makeSlidersConsistent()));
    }
    foreach(QSlider* slider, sliders->keys())
    {
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(makeSpinBoxesConsistent()));
    }
}

void ConfigurationWindow::closeEvent(QCloseEvent *)
{
    this->onAbortConfigurations();
}


}
