#include "pathplannerprm.h"
#include "data/action.h"
#include "pathPlanner/hybridAStar/hybridposeaction.h"

namespace PRM{

using namespace PathPlanner;
using namespace Data;

PathPlannerPRM::PathPlannerPRM(QObject *parent) :
    QThread(parent)
{
}


void PathPlannerPRM::calculateActions(PRMPath p, const Map &map){
    if(p.size()==0){
        Action* trasl = new Action(Action::Translation,1);
        ldbg<<"Empty path... traslation 1m"<<endl;

        //Call this->onPerformActionEM;
        emit sigPerformActionPP(trasl); ;
        return;
    }
    ldbg<<"Calculating actions for path..."<<endl;
    const PathNode* robotPose=map.lastRobotPose(Config::robotID);
    double theta=robotPose->theta();
    Pose destination(p.last()->x(),p.last()->y(),0.0);

    emit sigFrontierToReachPP(destination);

    //action
    for(int i =0; i<p.size()-1;i++)
    {
        Point* p1= p.at(i);
        Point* p2= p.at(i+1);
        double distance=p1->distance(*p2);
        //        ldbg<<"Path Planner PRM: iteration: "<<i<<endl;
        //        ldbg<<"From: "<<*p1<<" To: "<<*p2<<endl;
        Pose poseIni(p1->x(),p1->y(),theta);
        Pose poseFin(p2->x(),p2->y(),0.0);
        double angle2 = computeRotationFromPoses(poseIni,poseFin);
        Action* rotat = new Action(Action::Rotation,fromRadiantToDegree(angle2));
        emit sigPerformActionPP(rotat);
        theta=theta+angle2;
        Action* trasl = new Action(Action::Translation,distance);
        emit sigPerformActionPP(trasl);
        //        ldbg<<"Sending action: rotation "<<rotat->getValue()<<endl;
        //        ldbg<<"Sending action: traslation "<<trasl->getValue()<<endl;
    }

}


void PathPlannerPRM::noPathFound(){
    Action* trasl = new Action(Action::Translation,1);
    ldbg<<"Empty path... traslation 1m"<<endl;
    emit sigPerformActionPP(trasl);
}

}
