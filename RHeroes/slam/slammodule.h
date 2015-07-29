/*
 * slammodule.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef SLAMMODULE_H_
#define SLAMMODULE_H_


#include "engine/slamengine.h"
#include "shared/mutexqueue.h"
#include "data/odometrydata.h"
#include "data/buddymessage.h"

#include "PRM/prmalgorithm.h"
#include <QThread>
#include <QMutex>

namespace SLAM {

class SLAMModule : public QThread
{
    Q_OBJECT
public:
    explicit SLAMModule(uint robotId, const Data::Pose &initialPose);
    virtual ~SLAMModule();
    Map getMap();
    Map getMap(bool fullMap);
    uint getFrontierOwner(Geometry::Frontier frontier);
    //PRM
    PRM::PRMAlgorithm* getPRM();
    //

signals:
    void scanReceived();
    void odometryReceived();
    void insReceived();
    void mapKOReceived();
    void outdoorReceived();
    void newRobotPose(SLAM::TimedPose pose);
    //PRM
    void newMapPRM(Map map);
    //

public slots:
    void handleScan();
    void handleOdometry();
    void handleINS();
    void handleMapKO();
    void handleOutdoor();
    void onSensorData(const Data::Message &msg);
    void forwardRobotPose(TimedPose pose);

protected:
    virtual void run();

    struct TimedPointScan {
        double timestamp;
        Geometry::PointScan *scan;
    };

private:
    Shared::MutexQueue<TimedPointScan> scanQueue;
    Shared::MutexQueue<Data::OdometryData *> odoQueue;
    Shared::MutexQueue<Data::INSData *> insQueue;
    Engine::SLAMEngine *slam;
    QMutex mutex;
    uint robotId;
    QMap<uint,SLAM::Map*> robotsMap;
    void updateMap(uint mapId);

    //PRM
    PRM::PRMAlgorithm* prm;
    bool firstIteration;
    //
};

} /* namespace SLAM */

#endif /* SLAMMODULE_H_ */
