#ifndef PATHPLANNERPRM_H
#define PATHPLANNERPRM_H

#include <QObject>
#include "aStarPRM/astaralgorithmprm.h"
#include "data/action.h"


namespace PRM{

class PathPlannerPRM : public QThread
{
    Q_OBJECT
public:
    explicit PathPlannerPRM(QObject *parent = 0);
    void calculateActions(PRMPath p, const Map &map);
    void noPathFound();
    
signals:
    void sigPerformActionPP(PathPlanner::AbstractAction* action);
    void sigRestartExplorationPP();
    void sigFrontierToReachPP(Data::Pose);

public slots:
    
};

}
#endif // PATHPLANNERPRM_H
