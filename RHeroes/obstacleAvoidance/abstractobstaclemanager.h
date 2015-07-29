#ifndef ABSTRACTOBSTACLEMANAGER_H
#define ABSTRACTOBSTACLEMANAGER_H

#include <QObject>
#include <data/sonardata.h>


class AbstractObstacleManager : public QObject
{
public:
    AbstractObstacleManager(bool);
    virtual ~AbstractObstacleManager();
    void setStatus(bool);
    bool getStatus();
    void handleFrontSonarData(const Data::SonarData &sonar);
    void handleBackSonarData(const Data::SonarData &sonar);

private:
    bool status;
};


#endif // ABSTRACTOBSTACLEMANAGER_H
