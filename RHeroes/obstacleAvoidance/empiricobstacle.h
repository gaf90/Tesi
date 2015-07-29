#ifndef EMPIRICOBSTACLE_H
#define EMPIRICOBSTACLE_H

#include <QObject>
#include <QStack>
#include <QQueue>
#include <QImage>
#include "data/message.h"
#include "data/action.h"
#include "data/robotstate.h"
#include "data/laserdata.h"
#include "data/odometrydata.h"
#include "data/cameradata.h"
#include "data/statedata.h"
#include "data/sonardata.h"
#include "data/insdata.h"
#include "data/robotstate.h"
#include "data/waypointcommand.h"
#include "slam/timedpose.h"
#include "slam/slammodule.h"
#include <QMutex>
#include "logic/inversekinematic.h"
#include "pathPlanner/abstractaction.h"
#include "pathPlanner/hybridAStar/hybridposeaction.h"
#include "pathPlanner/hybridAStar/hybridastaralgorithm.h"
#include "logic/robot.h"
#include <QTimer>
#include <QTime>
#include "data/sonardata.h"
#include "abstractobstaclemanager.h"
#include <data/action.h>

class EmpiricObstacle: public AbstractObstacleManager
{
 public:
    EmpiricObstacle(bool isActive);

    void setStatus(bool);
    void handleFrontSonarData(const Data::SonarData&,QQueue<Data::Action *> normalActionQueue);
    void handleBackSonarData(const Data::SonarData&,QQueue<Data::Action *> normalActionQueue);
    void changeReactiveFSM(int);
    void handleFrontObstacle(const Data::SonarData &sonar);
    void handleBackObstacle(const Data::SonarData &sonar);
private:
    bool status;
};

#endif // EMPIRICOBSTACLE_H
