#include "sonarsensor.h"
#include "data/usarmessage.h"
#include "data/sonardata.h"
#include "slam/geometry/point.h"
#include <typeinfo>

#define JUST_A_BIG_NUMBER 10 //bigger then the maxRange of the SonarSensor in USARSim

QString minSonarName;
QString maxSonarName;

namespace Middleware{
SonarSensor::SonarSensor(QObject *parent) :
    Sensor(parent)
{
    for(int i = 0; i < SONAR_PER_SIDE; i++){
        backSonar[i] = FRONTSONAR_SAFE_DISTANCE; //setting sonars to standard distance
        frontSonar[i] = FRONTSONAR_SAFE_DISTANCE;
    }
    frontSonarPosition[0] = -0.30;
    frontSonarPosition[1] = -0.15;
    frontSonarPosition[2] = 0.0;
    frontSonarPosition[3] = 0.15;
    frontSonarPosition[4] = 0.30;
}

SonarSensor::~SonarSensor()
{
}

void SonarSensor::onMessageReceived(const Data::Message &message)
{
    if(typeid(message) == typeid(const Data::USARMessage &)) {
        const Data::USARMessage &msg = (const Data::USARMessage &) message;

        if(msg.getType() == "SEN" &&
                msg.contains("Type") &&
                (msg["Type"] == "Sonar")) {
            //            ldbg << "msg[\"range\"]"<<msg["Range"] << endl;
            double distance = msg["Range"].toDouble();
            Data::SonarData::SonarPosition sonarPosition;
            bool doEmit = false;
            if(msg["Name"] == "backSonarLL"){
                backSonar[0] = distance;
                sonarPosition = Data::SonarData::Back;
                doEmit = true;
            }
            if(msg["Name"] == "backSonarL"){
                backSonar[1] = distance;
                sonarPosition = Data::SonarData::Back;
                doEmit = true;
            }
            if(msg["Name"] == "backSonarM"){
                backSonar[2] = distance;
                sonarPosition = Data::SonarData::Back;
                doEmit = true;
            }
            if(msg["Name"] == "backSonarR"){
                backSonar[3] = distance;
                sonarPosition = Data::SonarData::Back;
                doEmit = true;
            }
            if(msg["Name"] == "backSonarRR"){
                backSonar[4] = distance;
                sonarPosition = Data::SonarData::Back;
                doEmit = true;
            }
            if(msg["Name"] == "frontSonarLL"){
                frontSonar[0] = distance;
                sonarPosition = Data::SonarData::Front;
                doEmit = true;
            }
            if(msg["Name"] == "frontSonarL"){
                frontSonar[1] = distance;
                sonarPosition = Data::SonarData::Front;
                doEmit = true;
            }
            if(msg["Name"] == "frontSonarM"){
                frontSonar[2] = distance;
                sonarPosition = Data::SonarData::Front;
                doEmit = true;
            }
            if(msg["Name"] == "frontSonarR"){
                frontSonar[3] = distance;
                sonarPosition = Data::SonarData::Front;
                doEmit = true;
            }
            if(msg["Name"] == "frontSonarRR"){
                frontSonar[4] = distance;
                sonarPosition = Data::SonarData::Front;
                doEmit = true;
            }

            if(doEmit == true){
                double minBackDistance;
                double maxBackDistance;
                double minFrontDistance;
                double maxFrontDistance;

                if(sonarPosition == Data::SonarData::Back){
                    minBackDistance = getMinDistance(backSonar);
                    maxBackDistance = getMaxDistance(backSonar);
                }else{

                    minFrontDistance = getMinDistance(frontSonar);
                    maxFrontDistance = getMaxDistance(frontSonar);
                }
                emit sigSensorData(Data::SonarData(frontSonar,backSonar, minSonarName,maxSonarName, minFrontDistance, maxFrontDistance, minBackDistance, maxBackDistance,sonarPosition, getSensedObstacle()));
            }

        }
    }
}

SLAM::Geometry::LineSegment SonarSensor::getSensedObstacle() const
{
    int obstacleStartIndex = 0;
    int obstacleEndIndex = SONAR_PER_SIDE - 1;

    for(int i = 0; i < SONAR_PER_SIDE; i++){
        if(frontSonar[i] < MIN_SAFE_BACKSONAR_DISTANCE){
            obstacleStartIndex = i;
            break;
        }
    }

    for(int i = SONAR_PER_SIDE - 1; i >= 0; i--){
        if(frontSonar[i] < MIN_SAFE_BACKSONAR_DISTANCE){
            obstacleEndIndex = i;
            break;
        }
    }
    if(obstacleStartIndex == obstacleEndIndex){
        //the obstacle must be a line, not a point, enlarge it
        if(obstacleStartIndex > 0 and obstacleEndIndex < (SONAR_PER_SIDE - 1)){
            if(frontSonar[obstacleStartIndex-1] > frontSonar[obstacleStartIndex+1])
                obstacleEndIndex++;
            else
                obstacleStartIndex--;
        }

        if(obstacleEndIndex == 0)
            obstacleEndIndex = 1;
        if(obstacleStartIndex == (SONAR_PER_SIDE -1))
            obstacleStartIndex = SONAR_PER_SIDE -2;
    }
    SLAM::Geometry::Point p1(frontSonarPosition[obstacleStartIndex], frontSonar[obstacleStartIndex] + X_TRANSLATION_SONAR_SICK);
    SLAM::Geometry::Point p2(frontSonarPosition[obstacleEndIndex], frontSonar[obstacleEndIndex] + X_TRANSLATION_SONAR_SICK );
    SLAM::Geometry::LineSegment obstacle (p1,p2);
//    ldbg << obstacle << endl;
    return obstacle;
}

double SonarSensor::getMinDistance(double sonarArray[]) const
{
    double minDistance = JUST_A_BIG_NUMBER;
    for(int i = 0; i < SONAR_PER_SIDE; i++){
        if(sonarArray[i] < minDistance){
            minDistance = sonarArray[i];
            if (i == 0)
                minSonarName = "SonarLL";
            if (i == 1)
                minSonarName = "SonarL";
            if (i == 2)
                minSonarName = "SonarM";
            if (i == 3)
                minSonarName = "SonarR";
            if (i == 4)
                minSonarName = "SonarRR";
        }
    }
    return minDistance;
}

double SonarSensor::getMaxDistance(double sonarArray[]) const
{
    double maxDistance = 0;
    for(int i = 0; i < SONAR_PER_SIDE; i++){
        if(sonarArray[i] > maxDistance){
            maxDistance = sonarArray[i];
            if (i == 0)
                maxSonarName = "SonarLL";
            if (i == 1)
                maxSonarName = "SonarL";
            if (i == 2)
                maxSonarName = "SonarM";
            if (i == 3)
                maxSonarName = "SonarR";
            if (i == 4)
                maxSonarName = "SonarRR";
        }
    }
    return maxDistance;
}

}

