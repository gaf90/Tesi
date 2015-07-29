#include <limits>
#include "transmissionprobabilitycriterion.h"
#include "criteriaName.h"
#include "data/action.h"
#include "shared/utilities.h"

#include "shared/config.h"

#define CUTOFF_THRESHOLD    12.650f

namespace Exploration{

using namespace SLAM::Geometry;
using namespace Data;

TransmissionProbabilityCriterion::TransmissionProbabilityCriterion(double weight, uint robotId) :
    Criterion(TRANSMISSION_PROBABILITY, weight, true), robotId(robotId)
{
}

TransmissionProbabilityCriterion::~TransmissionProbabilityCriterion()
{

}

double TransmissionProbabilityCriterion::computeAttenuation(const Pose& myPose, const Pose& otherPose, const Frontier *frontier){
    Point myPoint(myPose.getX(), myPose.getY());
    Point otherPoint(otherPose.getX(), otherPose.getY());
    double oldDistance = myPoint.distance(otherPoint);
    double newDistance = frontier->centroid().distance(otherPoint);
    if(oldDistance<0.0001){
        oldDistance=0.0001;
    }
    if(newDistance<0.0001){
        newDistance=0.0001;
    }
    //Compute attenuation
    //double attenuation = 10*N*log10(myDistaceFromBS/eDo) - 10*N*log10(frontierDistance/eDo);
    return log10(oldDistance/newDistance);
}

double TransmissionProbabilityCriterion::computePathLoss(double attenuationFactor,const Frontier *frontier, const SLAM::Map &map, QHash<uint, double> &powerSignalData){
    double minSignalStrength=-std::numeric_limits<double>::max();
    const Pose *myPose = map.lastRobotPose(robotId);
    if(myPose == NULL){
        return minSignalStrength;
    }
    foreach(uint key, powerSignalData.keys()){
        double currentSignalStrength=powerSignalData[key];
        if(currentSignalStrength!=currentSignalStrength){
            continue;
        }
        QList<SLAM::TimedPose> t=map.getRobotPath(key);
        if(t.isEmpty()){
            continue;
        }
        const Pose& otherPose = t.last();
        currentSignalStrength+=10*attenuationFactor*computeAttenuation(*myPose,otherPose,frontier);
        /*if(currentSignalStrength>=0){
            currentSignalStrength=-1;
        }*/
        //qDebug("- %f",currentSignalStrength);
        minSignalStrength=max(minSignalStrength,currentSignalStrength);
    }
    return minSignalStrength;
}

double TransmissionProbabilityCriterion::evaluate(const Frontier *frontier, const SLAM::Map &map,
                                                  int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(batteryTime)
    //obtaining some constants fron the config file
    double N = Config::attenuationFactor;
    double signalCutoff = Config::signalCutoff+CUTOFF_THRESHOLD;

    //qDebug("TRANSMISSION USED: %f %f %f %f",N,signalCutoff,frontier->centroid().x(),frontier->centroid().y());

    double pathloss = computePathLoss(N,frontier,map,powerSignalData);


    double p = (pathloss>=signalCutoff)?1.0f:0.0f;
    //qDebug("EXIT CLEAR: %f %f!",pathloss,p);
    return p;


}


}
