/*
 * datasetworker.h
 *
 *  Created on: 29/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef DATASETWORKER_H_
#define DATASETWORKER_H_

#include "slam/engine/slamengine.h"
#include "slam/engine/scanmatcherselection.h"
#include <QThread>
#include <QFile>
#include <QTimer>
#include <QMutex>
#include "slam/support/stopwatch.h"

namespace SLAM {
namespace Dataset {

class DatasetWorker : public QThread
{
    Q_OBJECT
public:
    enum SLAMEngineType {
        OdometryOnly,
        LegacyDeterministic,
        Deterministic,
        ISAM
    };

    enum DatasetFileFormat {
        USARSimDump,
        CARMENLog
    };

public:
    explicit DatasetWorker(QObject *parent = 0);
    virtual ~DatasetWorker();
    
    void setDelay(double delay);
    void setSLAMType(SLAMEngineType type);
    void setScanMatcher(Engine::ScanMatcherSelection matcher);
    void loadDataset(const QString &file);
    int minimumIterationAllowed() const;
    int maximumIterationAllowed() const;
    int iteration() const;
    QString datasetFileName() const;
    bool loaded() const;
    bool playing() const;
    bool hasMaps() const;
    Map previousMap();
    Map map();
    Geometry::PointScan scan();
    Eigen::Vector3d deltaOdometry();

public slots:
    void play();
    void pause();
    void stepForward(int steps);
    void stepBack(int steps);
    void stepForward();
    void stepBack();

private slots:
    void actualStepForward(int steps);

signals:
    void executionFinished();
    void requestStepForward(int steps);

protected:
    virtual void run();

private:
    void stepUpdate(const Geometry::PointScan &scan, int &stepsDone);

private:
    DatasetFileFormat format;
    bool isPlaying, connected, completed;
    int delay;
    int iter, realIter, minIter;
    Engine::SLAMEngine *slam;
    QTimer *timer;
    QFile *dataset;
    QString fileName;
    QMutex mutex;
    Map lastDroppedMap;
    Geometry::Rototranslation previousPose, currentPose;
    QList<Map> mapHistory;
    QList<Geometry::PointScan> scanHistory;
    QList<Eigen::Vector3d> deltaOdometryHistory;
    SLAMEngineType slamType;
    Engine::ScanMatcherSelection matcher;
    Support::Stopwatch stopwatch;
};

} /* namespace Dataset */
} /* namespace SLAM */

#endif /* DATASETWORKER_H_ */
