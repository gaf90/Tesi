#include "evaluationfunction.h"
#include "data/wirelessmessage.h"
#include "shared/config.h"
#include "shared/constants.h"
#include "slam/geometry/point.h"
#include "pathPlanner/pathplannerutils.h"

namespace Exploration{

using namespace Data;
using namespace SLAM;
using namespace SLAM::Geometry;

    EvaluationFunction::EvaluationFunction(uint robotId) :
        robotId(robotId)
    {
    }

    EvaluationFunction::~EvaluationFunction()
    {

    }

    double EvaluationFunction::computeInformationGain(const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map)
    {
        Q_UNUSED(map)
        //ldbg << "try to get the frontier length" << endl;
        //EasyWay: return the length of the frontier.
        return frontier.length();
    }

    double EvaluationFunction::computeDistanceFromOtherRobots(
            const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map, const uint robot)
    {
        //EasyWay: euclidean distance.
        const Pose *pose = map.lastRobotPose(robot);
        SLAM::Geometry::Point frontierCentroid = frontier.centroid();

        double dist = frontierCentroid.distance(SLAM::Geometry::Point(pose->getX(), pose->getY()));

        return dist;
    }



    double EvaluationFunction::computeTransmissionProbability(
            const SLAM::Geometry::Frontier &frontier, const SLAM::Map &map, const QHash<uint, double> &powerData)
    {
        //obtaining some constants fron the config file
        double N = Config::attenuationFactor;
        double signalCutoff = Config::signalCutoff;
        double eDo = Config::eDo;

        //compute distances
        const Pose *myPose = map.lastRobotPose(robotId);
        const Pose *bsPose = map.lastRobotPose(BASE_STATION_ID);
        Point myPoint(myPose->getX(), myPose->getY());
        Point bsPoint(bsPose->getX(), bsPose->getY());
        double myDistaceFromBS = myPoint.distance(bsPoint);
        double frontierDistance = frontier.centroid().distance(bsPoint);

        //Compute attenuation
        double attenuation = 10*N*log10(myDistaceFromBS/eDo) - 10*N*log10(frontierDistance/eDo);

        //search for the nearest neighbour
        double minSignalStrength = 0.0;
        uint nearestRobotId = BASE_STATION_ID;
        foreach(uint key, powerData.keys()){
            if(powerData[key] <= minSignalStrength && key!=robotId){
                nearestRobotId = key;
                minSignalStrength = powerData[key];
            }
        }
        double pathloss = minSignalStrength+attenuation;

        //finally, compute the probability:
        double p = 2*pow(10, (signalCutoff-pathloss));

        return p;


    }

    EvaluationRecords* EvaluationFunction::evaluateFrontiers(const QList<Frontier *> &frontiers, const SLAM::Map &map,
                                                 int batteryTime, QHash<uint, double> &powerSignalData)
    {
        //ldbg << "Evaluation function is evaluating..." << endl;
        EvaluationRecords *toRet = new EvaluationRecords(robotId);
        //int count = 0;
        //ldbg << "I have to evaluate "<<frontiers.size()<<" frontiers..." <<endl;
        foreach(Frontier *f, frontiers){
            //ldbg << count << "-th frontier to evaluate" << endl;
            Point posePoint(f->centroid().x(), f->centroid().y());
            bool evaluate = PathPlanner::frontierFound(map.frontiers(), posePoint);
            double value = 0.0;
            if(evaluate)
                value = evaluateFrontier(f, map, batteryTime, powerSignalData);
            //ldbg << "inserting the pair inside the map" << endl;
            //ldbg << "Frontier <" << f.centroid().x() << ", " << f.centroid().y() << "> with value = " << value << endl;
            toRet->putEvaluation(*f, value);
            //count++;
        }
//        foreach(Frontier f, *toRet->getFrontiers()){
//            ldbg << "Frontier <" << f.centroid().x() << ", " << f.centroid().y() << "> with value = " << toRet->getEvaluation(f) << endl;
//        }
        toRet->normalize();
//        foreach(Frontier f, *toRet->getFrontiers()){
//            ldbg << "Frontier <" << f.centroid().x() << ", " << f.centroid().y() << "> with value = " << toRet->getEvaluation(f) << endl;
//        }
//        ldbg << "evaluation records filled" << endl;
        return toRet;
    }

//        //Search for the nearest neighbour w.r.t. the signal power.
//        int maxRobot=Config::robotCount, robotIndex = 0;

//        double maxStrength = 0.0;
//        for(int i=0; i<maxRobot; i++){
//            //Ask the WSS the signal power between the robot and the base station
//            double Sd0 = 0.0;
//            //Compute the distance between
//            double d0 = 0.0;
//            //Compute the distance between the frontier and the robot
//            double d = 0.0;
//            //compute the signal level
//            double signal = Sd0 - 10 * log(d/d0);

//            if(signal > maxStrength){
//                maxStrength = signal;
//                robotIndex = i;
//            }
//        }

//        //Check the power w.r.t the base station
//        double Sd0 = 0.0;
//        double d0 = 0.0;
//        double d = 0.0;
//        double signal = Sd0 - 10 * Config::attenuationFactor * log(d/d0);

//        if(signal > maxStrength){
//            maxStrength = signal;
//            robotIndex = BASE_STATION_ID;
//        }

//        return 0.0;

}
