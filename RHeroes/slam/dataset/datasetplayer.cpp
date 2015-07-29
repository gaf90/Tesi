/*
 * datasetplayer.cpp
 *
 *  Created on: 28/nov/2012
 *      Author: Mladen Mazuran
 */

#include "datasetplayer.h"
#include "ui_datasetplayer.h"
#include "datasetconstants.h"
#include "exporter.h"
#include "data/usarmessage.h"
#include "slam/geometry/segmentscan.h"
#include <cmath>
#include <QFileDialog>
#include <QList>
#include <QKeyEvent>

namespace SLAM {
namespace Dataset {

using namespace Geometry;
using namespace Data;

DatasetPlayer::DatasetPlayer(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::DatasetPlayer), timer(new QTimer()), displayedIteration(-1)
{
    ui->setupUi(this);
    ui->scanView->setKeepRatio(true);
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_event()));
    connect(&worker, SIGNAL(executionFinished()), this, SLOT(interfaceUpdate()), Qt::QueuedConnection);
    timer->setInterval(DS_REFRESH_RATE);
    timer->start();
    ui->actionSkipBack->setEnabled(false);
    ui->actionBack->setEnabled(false);
    ui->actionPlay->setEnabled(false);
    ui->actionForward->setEnabled(false);
    ui->actionSkipForward->setEnabled(false);
    ui->actionReload->setEnabled(false);
}

DatasetPlayer::~DatasetPlayer()
{
    delete ui;
    delete timer;
}

void DatasetPlayer::interfaceUpdate()
{
    int iter = worker.iteration();
    int lmin = worker.minimumIterationAllowed();
    int lmax = worker.maximumIterationAllowed();
    if(iter == -1) {
        ui->iterationValue->setText("0");
    } else {
        ui->iterationValue->setText(QString::number(iter));
    }

    if(ui->actionBack->isEnabled()) {
        if(iter - 1 < lmin) ui->actionBack->setEnabled(false);
    } else {
        if(iter - 1 >= lmin) ui->actionBack->setEnabled(true);
    }

    if(ui->actionSkipBack->isEnabled()) {
        if(iter - 10 < lmin) ui->actionSkipBack->setEnabled(false);
    } else {
        if(iter - 10 >= lmin) ui->actionSkipBack->setEnabled(true);
    }

    if(ui->actionForward->isEnabled() && lmax > -1) {
        if(iter + 1 > lmax) {
            ui->actionPlay->setEnabled(false);
            ui->actionForward->setEnabled(false);
        }
    } else {
        if(iter + 1 <= lmax || lmax == -1) {
            ui->actionPlay->setEnabled(true);
            ui->actionForward->setEnabled(true);
        }
    }

    if(ui->actionSkipForward->isEnabled() && lmax > -1) {
        if(iter + 10 > lmax) ui->actionSkipForward->setEnabled(false);
    } else {
        if(iter + 10 <= lmax || lmax == -1) ui->actionSkipForward->setEnabled(true);
    }

    if(!worker.playing())
        pauseUpdateIcon();
}

void DatasetPlayer::pauseUpdate()
{
    worker.pause();
}

void DatasetPlayer::pauseUpdateIcon()
{
    ui->actionPlay->setIcon(QIcon(":/images/play.svg"));
}

void DatasetPlayer::application_quit()
{
    close();
}

void DatasetPlayer::on_checkOdometryOnly_toggled(bool checked)
{
    if(checked)
        worker.setSLAMType(DatasetWorker::OdometryOnly);
}

void DatasetPlayer::on_checkLegacy_toggled(bool checked)
{
    if(checked)
        worker.setSLAMType(DatasetWorker::LegacyDeterministic);
}

void DatasetPlayer::on_checkDeterministic_toggled(bool checked)
{
    if(checked)
        worker.setSLAMType(DatasetWorker::Deterministic);
}

void DatasetPlayer::on_checkISAM_toggled(bool checked)
{
    if(checked)
        worker.setSLAMType(DatasetWorker::ISAM);
}

void DatasetPlayer::on_checkLiGriffithsICL_toggled(bool checked)
{
    if(checked)
        worker.setScanMatcher(Engine::LiGriffithsICLSelection);
}

void DatasetPlayer::on_checkClassicICL_toggled(bool checked)
{
    if(checked)
        worker.setScanMatcher(Engine::ClassicICLSelection);
}

void DatasetPlayer::on_checkFilteredICL_toggled(bool checked)
{
    if(checked)
        worker.setScanMatcher(Engine::FilteredICLSelection);
}

void DatasetPlayer::on_checkRANSACMatcher_toggled(bool checked)
{
    if(checked)
        worker.setScanMatcher(Engine::RANSACMatcherSelection);
}

void DatasetPlayer::on_delaySlider_valueChanged(int value)
{
    const int szero = 0;            // slider zero = 0
    const int smin  = szero + 1;    // slider min  = 1
    const int smax  = 99;           // slider max  = 99
    const double dmin = 0.01;       // delay min   = 10ms
    const double dmax = 5;          // delay max   = 5s
    double delay;

    if(value == szero) {
        delay = 0;
    } else {
        const double l1 = std::log10(dmin);
        const double l2 = std::log10(dmax);
        const double a  = (l1 - l2) / (smin - smax);
        const double b  = (l2 * smin - l1 * smax) / (smin - smax);
        delay = std::pow(10, a * value + b);
    }

    QString um = (delay < 0.5 ? "ms" : "s");
    double printvalue = (delay < 0.5 ? round(delay * 1000) : delay);
    QString formatted = QString::number(printvalue, 'g', 4) + " " + um;

    ui->delayValue->setText(formatted);
    worker.setDelay(delay);
}
void DatasetPlayer::on_actionPlay_triggered()
{
    if(worker.playing()) {
        pauseUpdateIcon();
        worker.pause();
    } else {
        ui->actionPlay->setIcon(QIcon(":/images/pause.svg"));
        worker.play();
    }
}

void DatasetPlayer::on_actionForward_triggered()
{
    if(worker.playing()) worker.pause();
    worker.stepForward();
}

void DatasetPlayer::on_actionSkipForward_triggered()
{
    if(worker.playing()) worker.pause();
    worker.stepForward(10);
}

void DatasetPlayer::on_actionSkipBack_triggered()
{
    if(worker.playing()) worker.pause();
    worker.stepBack(10);
}

void DatasetPlayer::on_actionBack_triggered()
{
    if(worker.playing()) worker.pause();
    worker.stepBack();
}

void DatasetPlayer::on_actionQuit_triggered()
{
    if(worker.playing()) worker.pause();
    worker.exit(0);
    QTimer::singleShot(500, this, SLOT(application_quit()));
}

void DatasetPlayer::on_actionOpen_triggered()
{
    QString file = QFileDialog::getOpenFileName(this, "Open dataset", "",
                        "All supported datasets (*.txt *.log *.clf);;"
                        "USARSim datasets (*.txt *.log);;"
                        "CARMEN Log Files (*.clf)");
    if(!file.isEmpty()) {
        if(worker.playing()) worker.pause();
        ui->openedDataset->setText(file);
        worker.loadDataset(file);
        ui->mapView->reset();
        ui->scanView->reset();
        ui->actionReload->setEnabled(true);
        interfaceUpdate();
    }
}

void DatasetPlayer::on_actionReload_triggered()
{
    if(worker.playing()) worker.pause();
    worker.loadDataset(worker.datasetFileName());
    ui->mapView->reset();
    ui->scanView->reset();
    interfaceUpdate();
}

void DatasetPlayer::on_mapView_zoomChanged(double zoom)
{
    setWindowTitle(QString("Dataset player (zoom: %1x)").arg(zoom * 100, 0, 'g', 4));
}

void DatasetPlayer::on_mapExport_clicked()
{
    pdfSave(ui->mapView);
}

void DatasetPlayer::on_scanExport_clicked()
{
    pdfSave(ui->scanView);
}

void DatasetPlayer::on_codeExport_clicked()
{
    QString file = QFileDialog::getSaveFileName(this, "Save C++ header file", "",
                        "C++ header files (*.h *.hpp *.hxx)");
    if(!file.isEmpty()) {
        Exporter::exportCPPCode(file, &worker);
    }
}

void DatasetPlayer::pdfSave(const MapViewWidget *map)
{
    QString file = QFileDialog::getSaveFileName(this, "Save PDF document", "",
                        "PDF files (*.pdf)");
    if(!file.isEmpty()) {
        Exporter::exportPDF(file, map);
    }
}


void DatasetPlayer::timer_event()
{
    int iteration = worker.iteration();
    if(worker.hasMaps() && displayedIteration != iteration) {
        ui->mapView->setMap(worker.map());
        ui->scanView->setScan(worker.scan());
        displayedIteration = iteration;
        interfaceUpdate();
    }
}

void DatasetPlayer::keyPressEvent(QKeyEvent *event)
{
    /* Workaround for space */
    if(event->key() == ' ' && ui->actionPlay->isEnabled()) {
        on_actionPlay_triggered();
    }
    QMainWindow::keyPressEvent(event);
}

void DatasetPlayer::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event)
    on_actionQuit_triggered();
}

} /* namespace Dataset */
} /* namespace SLAM */
