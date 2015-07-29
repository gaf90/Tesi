#include "batterycriterionprm.h"

#include "CriteriaNamePRM.h"
#include "data/action.h"

namespace PRM{

using namespace SLAM::Geometry;
using namespace SLAM;
using namespace Data;

BatteryCriterionPRM::BatteryCriterionPRM(double weight, uint robotId) :
    CriterionPRM(BATTERY_PRM, weight, true), robotId(robotId)
{
}

BatteryCriterionPRM::~BatteryCriterionPRM()
{

}

double BatteryCriterionPRM::evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                                     int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(frontier) Q_UNUSED(map) Q_UNUSED(powerSignalData)

    QList<double> remainingBattery;
    foreach(PRMPath p, paths){
        const Pose *pose = map.lastRobotPose(robotId);
        double theta=pose->getTheta();

        double timeRequired=0.0;
        for(int i =0; i<p.size()-1;i++){
            Point* p1= p.at(i);
            Point* p2= p.at(i+1);
            double distance=p1->distance(*p2);
            Action trasl;
            trasl.setType(Action::Translation);
            trasl.setValue(distance);
            Action rotat;
            rotat.setType(Action::Rotation);
            Pose poseIni(p1->x(),p1->y(),theta);
            Pose poseFin(p2->x(),p2->y(),0.0);
            double angle2 = computeRotationFromPoses(poseIni,poseFin);
            rotat.setValue(fromRadiantToDegree(angle2));
            double traslTime = trasl.getTimeEstimate();
            double rotTime = rotat.getTimeEstimate();
            timeRequired = timeRequired+traslTime+rotTime;
            theta=theta+angle2;
            //ldbg<<"battery,path step "<<i<<" : "<<traslTime<<" + "<<rotTime<<endl;
        }
        double temp = batteryTime - timeRequired;
        remainingBattery.append(temp);
    }
    insertEvaluation(frontier,remainingBattery);
    return 0.0;
}



}
