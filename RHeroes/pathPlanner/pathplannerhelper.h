#ifndef PATHPLANNERHELPER_H
#define PATHPLANNERHELPER_H

#include <QObject>



namespace PathPlanner{

class PathPlannerModule;

class PathPlannerHelper : public QObject
{
    Q_OBJECT
public:
    explicit PathPlannerHelper(PathPlannerModule *ppmodule, QObject *parent = 0);
    virtual ~PathPlannerHelper();
signals:

public slots:
    void sendMessageToGUI();
    
private:
    PathPlannerModule *ppmodule;

};
}

#endif // PATHPLANNERHELPER_H
