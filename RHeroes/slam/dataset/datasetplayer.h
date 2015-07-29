/*
 * datasetplayer.h
 *
 *  Created on: 28/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef DATASETPLAYER_H_
#define DATASETPLAYER_H_

#include "datasetworker.h"
#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>

namespace Ui {
class DatasetPlayer;
}

namespace SLAM {
namespace Dataset {

class MapViewWidget;

class DatasetPlayer : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit DatasetPlayer(QWidget *parent = 0);
    ~DatasetPlayer();

private slots:
    void on_delaySlider_valueChanged(int value);
    void on_actionPlay_triggered();
    void on_actionForward_triggered();
    void on_actionSkipForward_triggered();
    void on_actionSkipBack_triggered();
    void on_actionBack_triggered();
    void on_actionQuit_triggered();
    void on_actionOpen_triggered();
    void on_actionReload_triggered();
    void on_mapView_zoomChanged(double zoom);
    void on_mapExport_clicked();
    void on_scanExport_clicked();
    void on_codeExport_clicked();
    void on_checkOdometryOnly_toggled(bool checked);
    void on_checkLegacy_toggled(bool checked);
    void on_checkDeterministic_toggled(bool checked);
    void on_checkISAM_toggled(bool checked);
    void on_checkLiGriffithsICL_toggled(bool checked);
    void on_checkClassicICL_toggled(bool checked);
    void on_checkFilteredICL_toggled(bool checked);
    void on_checkRANSACMatcher_toggled(bool checked);
    void timer_event();
    void application_quit();
    void interfaceUpdate();

private:
    void pauseUpdate();
    void pauseUpdateIcon();
    void pdfSave(const MapViewWidget *map);

protected:
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void closeEvent(QCloseEvent *event);

private:
    Ui::DatasetPlayer *ui;
    QTimer *timer;
    DatasetWorker worker;
    int displayedIteration;
};

} /* namespace Dataset */
} /* namespace SLAM */

#endif /* DATASETPLAYER_H_ */
