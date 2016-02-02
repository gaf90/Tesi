#include "shared/config.h"

#ifndef POARET_EXE
#   define POARET_EXE "Poaret"
#endif

#define SQUARE(x) ((x) * (x))

namespace Config {

int robotID = BASE_STATION_ID;

QString poaretBinary = POARET_EXE;

QString usarAddress = "127.0.0.1";
QString upisAddress = "127.0.0.1";
QString wssAddress = "127.0.0.1";

quint16 usarPort = 3000;
quint16 upisPort = 5003;
quint16 wssPort = 50000;

int robotCount;

Data::Pose baseStationPose(0,0,0);

Exploration::EvalType policy;

double attenuationFactor = 1.0;

double signalCutoff = -93;

double eDo = 2;

double HMin = 0, SMin = 210, VMin = 220, HMax = 20, SMax = 255, VMax = 255;

double robotMaxSpeed = 1.29;

int robotType = 0;

namespace SLAM {
double maxSpatialErrorPerMeter              = .1;
double maxAngularErrorPerMeter              = .1;
double maxSpatialErrorPerRadian             = 1;
double maxAngularErrorPerRadian             = 1;

double lookupThresholdElseberg              = 0.5;
double lookupThresholdAmigoni               = 0.5;
double lookupThresholdLiGriffiths           = 50;
double lookupThresholdProbabilistic         = 15;
double lookupThresholdPoseCentric           = 14;

double broadLookupThresholdElseberg         = 1.5;
double broadLookupThresholdAmigoni          = 1.5;
double broadLookupThresholdLiGriffiths      = 150;
double broadLookupThresholdProbabilistic    = 20;
double broadLookupThresholdPoseCentric      = 20; // 14;

int ransacMaximumIterations                 = 250;
int ransacMinimumIterations                 = 50;

double odometryMinVarianceX                 = 1e-4;
double odometryMinVarianceY                 = 1e-4;
double odometryMinVarianceTheta             = SQUARE(M_PI / 120);

double laserRangeVariance                   = SQUARE(0.01);
double laserAngleVariance                   = SQUARE(0.001);
double laserOutOfRange                      = 20; // 8

double splitThreshold                       = 0.04; //3 * std::sqrt(laserRangeVariance);
double mergeThreshold                       = 0.05; //3 * std::sqrt(laserRangeVariance);
double collinearityThreshold                = 10. / 180. * M_PI;
double thinningThreshold                    = 0.35;

double landmarkSpatialDistance              = 1;
double landmarkAngularDistance              = 30. / 180. * M_PI;
} /* namespace SLAM */

//PRM
namespace PRM {
double edgeThreshold = 4;
double movementRadius = 0.3;
int precision = 10000;
int pointNumber = 3000;
int pointNumberFrontier = 5;
int maxPointDistance = 20;
int pathNumber = 3;
}
//Obstacle
namespace OBS{

int obstacle_algorithm = 0;

int emp_angle_tolerance = 15;
double emp_straight_meters = 1;
double emp_back_meters = -0.3;
double emp_sonar_threshold = 0.22;
double dwa_laser_threshold = 0.5;
double dwa_laser_max_range = 10;

double dwa_time = 1;
double dwa_min_velocity = 0;
double dwa_max_velocity = 0.7;
double dwa_step = 0.5;
double dwa_safety = 20;
double dwa_pose_threshold = 0.1;
double dwa_sigma = 1;
double dwa_alpha_target = 0.5;
double dwa_beta_clearance = 1;
double dwa_gamma_velocity =0.2;

}

}
