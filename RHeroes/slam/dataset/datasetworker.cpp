/*
 * datasetwoker.cpp
 *
 *  Created on: 29/nov/2012
 *      Author: Mladen Mazuran
 */

#include "datasetworker.h"
#include "datasetconstants.h"
#include "debugconfiguration.h"
#include "data/usarmessage.h"
#include "slam/engine/odometryonlyslam.h"
#include "slam/engine/deterministicslam.h"
#ifdef USE_ISAM
#   include "slam/engine/isamslam.h"
#endif
#include "slam/legacy/iclslam.h"

namespace SLAM {
namespace Dataset {

using namespace ScanMatching;
using namespace Geometry;
using namespace Data;

DatasetWorker::DatasetWorker(QObject *parent) :
    QThread(parent), isPlaying(false), connected(false), completed(false),
    delay(0), iter(-1), realIter(-1), minIter(0), slam(NULL),
    timer(new QTimer(this)), dataset(NULL), slamType(Deterministic),
    matcher(Engine::RANSACMatcherSelection)
{
    moveToThread(this);
    start();
}

DatasetWorker::~DatasetWorker()
{
    delete dataset;
    delete timer;
    delete slam;
}

void DatasetWorker::run()
{
    timer = new QTimer(this);
    timer->setSingleShot(true);
    timer->setInterval(delay);

    exec();

    delete timer;
    timer = NULL;
}

void DatasetWorker::setDelay(double delay)
{
    this->delay = round(delay * 1000);
    timer->setInterval(this->delay);
    if(isPlaying) {
        timer->start();
    }
}

void DatasetWorker::setSLAMType(SLAMEngineType type)
{
    slamType = type;
}

void DatasetWorker::setScanMatcher(Engine::ScanMatcherSelection matcher)
{
    this->matcher = matcher;
}

void DatasetWorker::loadDataset(const QString &file)
{
    mutex.lock();

    delete dataset;
    delete slam;

    switch(slamType) {
    case OdometryOnly:
        slam = new Engine::OdometryOnlySLAM(0, Pose(0, 0, 0));
        break;
    case LegacyDeterministic:
        slam = new Legacy::ICLSLAM(0, Pose(0, 0, 0));
        break;
    case Deterministic:
        slam = new Engine::DeterministicSLAM(0, Pose(0, 0, 0), matcher);
        break;
    case ISAM:
#ifdef USE_ISAM
        slam = new Engine::ISAMSLAM(0, Pose(0, 0, 0), matcher);
#endif
        break;
    }

    if(file.endsWith(".clf", Qt::CaseInsensitive)) {
        format = CARMENLog;
    } else {
        format = USARSimDump;
    }

    dataset = new QFile(file);
    fileName = file;
    dataset->open(QIODevice::ReadOnly);
    minIter = 0;
    iter = -1;
    realIter = -1;
    mapHistory.clear();
    scanHistory.clear();

    if(!connected) {
        connect(timer, SIGNAL(timeout()), this, SLOT(stepForward()));
        connect(this, SIGNAL(requestStepForward(int)), this, SLOT(actualStepForward(int)));
        connected = true;
    }

    stopwatch.reset();

    completed = false;
    mutex.unlock();
}

void DatasetWorker::play()
{
    isPlaying = true;
    stepForward();
}

void DatasetWorker::pause()
{
    mutex.lock();
    isPlaying = false;
    timer->stop();
    mutex.unlock();
}

void DatasetWorker::actualStepForward(int steps)
{
    QMutexLocker locker(&mutex);
    int stepsDone = 0;
    while(stepsDone < steps && ((dataset && !dataset->atEnd()) || iter < realIter)) {
        if(iter < realIter) {
            iter++;
            stepsDone++;
            continue;
        }

        QString line = dataset->readLine();
        if(format == USARSimDump) {
            USARMessage m(line);
            if(m.getType() == "SEN" && m.contains("Type")) {
#if defined(GROUNDTRUTH_SOURCE) && GROUNDTRUTH_SOURCE
                if(m["Type"] == "GroundTruth" && m.contains("Location") && m.contains("Orientation")) {
                    QStringList locList = m["Location"].split(",");
                    QStringList oriList = m["Orientation"].split(",");
                    double gnd_x = locList[0].toDouble(), gnd_y = locList[1].toDouble();
                    double gnd_t = oriList[2].toDouble();
                    currentPose = Pose(gnd_y, gnd_x, -gnd_t);
                    stopwatch.start();
                    slam->handleOdometry(m["Time"].toDouble(), currentPose.vectorForm());
                    stopwatch.stop();
                } else
#elif defined(ODOMETRY_SOURCE) && ODOMETRY_SOURCE
                if(m["Type"] == "Odometry" && m.contains("Pose")) {
                    QStringList list = m["Pose"].split(",");
                    double odo_x = list[0].toDouble(), odo_y = list[1].toDouble();
                    double odo_t = list[2].toDouble();
                    currentPose = Pose(odo_y, odo_x, -odo_t);
                    stopwatch.start();
                    slam->handleOdometry(m["Time"].toDouble(), currentPose.vectorForm());
                    stopwatch.stop();
                } else
#endif
                if(m["Type"] == "INS") {
                    QString location = m["Location"];
                    QString orientation = m["Orientation"];
                    QStringList list = location.split(",");

                    double timestamp = m["Time"].toDouble();

                    /* Get the doubles associated to x, y, theta */
                    double ins_x = list[0].toDouble();
                    double ins_y = list[1].toDouble();
                    double ins_z = list[2].toDouble();

                    list = orientation.split(",");
                    double orientation_r = list[0].toDouble();
                    double orientation_p = list[1].toDouble();
                    double orientation_y = list[2].toDouble();
                    stopwatch.start();
#if !defined(ODOMETRY_SOURCE) || !ODOMETRY_SOURCE
                    currentPose = Pose(ins_y, ins_x, -orientation_y);
                    slam->handleOdometry(timestamp, currentPose.vectorForm());
#endif
                    slam->handleINS(INSData(timestamp,
                        Pose3D(ins_y, ins_x, -ins_z, orientation_r, orientation_p, -orientation_y)));
                    stopwatch.stop();
                } else if(m["Type"] == "RangeScanner") {
                    QList<double> readings;
                    double timestamp = m["Time"].toDouble();
                    double fov = m["FOV"].toDouble();
                    double resolution = m["Resolution"].toDouble();
                    const QStringList strReadings = m["Range"].split(",");
                    fforeach(const QString &num, strReadings) {
                        readings.append(num.toDouble());
                    }
                    PointScan scan(LaserData(timestamp, fov, resolution, readings));
                    stopwatch.start();
                    slam->handleScan(timestamp, scan);
                    stopwatch.stop();
                    stepUpdate(scan, stepsDone);
                    break; // 1 scan = 1 iteration
                }
            }
        } else if(format == CARMENLog) {
            if(line.startsWith("FLASER")) {
                QList<double> readings;
                QStringList list = line.split(' ');
                int nreadings = list[1].toInt();
                for(int i = 2; i < nreadings + 2; i++) {
                    readings.append(list[i].toDouble());
                }
                double timestamp = list[nreadings + 8].toDouble();
                double fov = M_PI;
                double resolution = fov / (nreadings - 1);
                double x = list[nreadings + 2].toDouble();
                double y = list[nreadings + 3].toDouble();
                double t = list[nreadings + 4].toDouble();
                PointScan scan(LaserData(timestamp, fov, resolution, readings));
                currentPose = Pose(x, y, t - M_PI_2);
                stopwatch.start();
                slam->handleOdometry(timestamp, currentPose.vectorForm());
                slam->handleScan(timestamp, scan);
                stopwatch.stop();
                stepUpdate(scan, stepsDone);
                break; // 1 scan = 1 iteration
            }
        }
    }

    if(!dataset || dataset->atEnd()) {
        delete dataset;
        dataset = NULL;
        isPlaying = false;
        completed = true;
        lprint << stopwatch.time() << endl;
        emit executionFinished();
    } else if(isPlaying) {
        timer->start();
    }
}

void DatasetWorker::stepUpdate(const PointScan &scan, int &stepsDone)
{
    stepsDone++;
    realIter++;
    iter++;
    if(mapHistory.size() >= DS_PLAY_MEMORY) {
        lastDroppedMap = mapHistory.first();
        mapHistory.removeFirst();
        scanHistory.removeFirst();
        deltaOdometryHistory.removeFirst();
        minIter++;
    }
    mapHistory.append(slam->getMap());
    scanHistory.append(scan);
    deltaOdometryHistory.append(previousPose.inverse() * currentPose);
    previousPose = currentPose;
}

void DatasetWorker::stepForward(int steps)
{
    emit requestStepForward(steps);
}

void DatasetWorker::stepBack(int steps)
{
    QMutexLocker locker(&mutex);
    iter = std::max(minIter, iter - steps);
}

void DatasetWorker::stepForward()
{
    emit requestStepForward(1);
}

void DatasetWorker::stepBack()
{
    stepBack(1);
}

int DatasetWorker::iteration() const
{
    return iter;
}

bool DatasetWorker::playing() const
{
    return isPlaying;
}

bool DatasetWorker::loaded() const
{
    return dataset != NULL;
}

bool DatasetWorker::hasMaps() const
{
    return mapHistory.size() > 0 && scanHistory.size() > 0;
}

int DatasetWorker::minimumIterationAllowed() const
{
    return minIter;
}

int DatasetWorker::maximumIterationAllowed() const
{
    return completed ? realIter : -1;
}

QString DatasetWorker::datasetFileName() const
{
    return fileName;
}

Map DatasetWorker::previousMap()
{
    QMutexLocker locker(&mutex);
    if(iter - minIter - 1 < 0) {
        return lastDroppedMap;
    } else {
        return mapHistory[iter - minIter - 1];
    }
}

Map DatasetWorker::map()
{
    QMutexLocker locker(&mutex);
    return mapHistory[iter - minIter];
}

PointScan DatasetWorker::scan()
{
    QMutexLocker locker(&mutex);
    return scanHistory[iter - minIter];
}

Eigen::Vector3d DatasetWorker::deltaOdometry()
{
    QMutexLocker locker(&mutex);
    return deltaOdometryHistory[iter - minIter];
}

} /* namespace Dataset */
} /* namespace SLAM */
