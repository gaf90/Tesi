#include "distancecriterionprm.h"
#include "CriteriaNamePRM.h"

namespace PRM{

using namespace SLAM::Geometry;
using namespace SLAM;

DistanceCriterionPRM::DistanceCriterionPRM(double weight, uint robotId)
    : CriterionPRM(DISTANCE_PRM, weight, false), robotId(robotId)
{
}

DistanceCriterionPRM::~DistanceCriterionPRM()
{

}

double DistanceCriterionPRM::evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)
    QList<double> distance = QList<double>();
    foreach (PRMPath p, paths){
        distance.append(pathLength(p));
    }
    insertEvaluation(frontier, distance);
    return 0.0;
}

double DistanceCriterionPRM::pathLength(PRMPath path){
    double length;
    for(int i=0; i<path.length()-1;i++){
        Point* p1=path.at(i);
        Point* p2=path.at(i+1);
        double distance=p1->distance(*p2);
        length=length+distance;
    }
    return length;
}

}
