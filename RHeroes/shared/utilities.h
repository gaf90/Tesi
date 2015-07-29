#ifndef UTILITIES_H
#define UTILITIES_H

#include "shared/constants.h"
#include <cmath>
#include <QString>
#include <QStringList>
#include "slam/geometry/point.h"
#include "data/pose.h"
#include "shared/logger.h"

/**
 * This function calculates the maximum beetween two values of type T
 *
 * @param a First value
 * @param b Second value
 * @return max{a,b}
 */
template <typename T> static inline T max(T a, T b) {
    return a > b ? a : b;
}
template <typename T> static inline T max(T a, T b, T c) {
    return max(max(a, b), c);
}
template <typename T> static inline T max(T a, T b, T c, T d) {
    return max(max(a, b), max(c, d));
}
template <typename T> static inline T max(T a, T b, T c, T d, T e) {
    return max(max(a, b), max(c, d), e);
}

/**
 * This function calculates the minimum beetween two values of type T
 *
 * @param a First value
 * @param b Second value
 * @return min{a,b}
 */
template <typename T>
static inline T min(T a, T b) {
    return a < b ? a : b;
}
template <typename T> static inline T min(T a, T b, T c) {
    return min(min(a, b), c);
}
template <typename T> static inline T min(T a, T b, T c, T d) {
    return min(min(a, b), min(c, d));
}
template <typename T> static inline T min(T a, T b, T c, T d, T e) {
    return min(min(a, b), min(c, d), e);
}


template <typename T, int N>
static inline int arraysize(const T (&arr)[N]) {
	return sizeof(arr) / sizeof(T);
}

/**
 * Exchange the value of the two variables @c a and @c b
 *
 * @param a First value
 * @param b Second value
 */
template <typename T>
static inline void exchange(T &a, T &b) {
    T temp = a;
    a = b;
    b = temp;
}

/**
 * This function wraps an angle that is bigger than 360�
 * and returns its equivalent within 0 and 360�
 * Or it wraps the angle if it is smaller tha -360�
 * and returns its equivalent within -360� and 0�
 *
 * @param angle that must be wrapped
 * @return a double that represents the wrapped angle
 */
static inline double wrapDeg(double angle) {

    while(angle < - FULL_ROUND_DEG) angle += FULL_ROUND_DEG;
    while(angle > FULL_ROUND_DEG) angle -= FULL_ROUND_DEG;
    return angle;
}

/**
 * This function wraps an angle that is bigger than 2*pi
 * and returns its equivalent within 0 and 2*pi
 * Or it wraps the angle if it is smaller than -2*pi
 * and returns its equivalent within -2*pi and 0
 *
 * @param angle that must be wrapped
 * @return a double that represents the wrapped angle
 */
static inline double wrapRad(double ang) {
    while(ang > M_PI) ang -= 2 * M_PI;
    while(ang <= -M_PI) ang += 2 * M_PI;
    return ang;
}


/**
 * This function convert an angle from its representation
 * in degree to the one in radiant.
 *
 * @param angle that must be converted
 * @return a double that is the converted angle
 */
static inline double fromDegreeToRadiants(double angle)
{
    return wrapDeg(angle)*M_PI/HALF_ROUND_DEG;
}

/**
 * This function convert an angle from its representation
 * in radiant to the one in degree.
 *
 * @param angle that must be converted
 * @return a double that is the converted angle
 */
static inline double fromRadiantToDegree(double angle)
{
    return angle*HALF_ROUND_DEG/M_PI;
}

static inline bool almostEqual(double v1, double v2, double threshold = 1e-9)
{
    double diff = v1 - v2;
    return diff > - threshold && diff < threshold;
}

/**
 * Generates the robot name from a unique index
 *
 * @param idx The unique index
 * @return Robot name in string format
 */
static inline QString robotNameFromIndex(uint idx) {
    if(idx == BASE_STATION_ID) {
        return QString("Robot_666");
    } else {
        return QString("Robot_%1").arg(idx);
    }
}

static inline uint robotIndexFromName(QString name){
    if(name == "Robot_666"){
        return BASE_STATION_ID;
    } else {
        QStringList list = name.split("_");
        return list[1].toUInt();
    }
 }

static inline double angularDistance(double start, double target)
{
    return wrapRad(target-start);
}

static inline double computeRotationFromPoses(const Data::Pose &actual, const Data::Pose &pointToReach){
//    OLD CODE
//
//    SLAM::Geometry::Point myPoint(actual.getX(), actual.getY());
//    SLAM::Geometry::Point goalPoint(pointToReach.getX(), pointToReach.getY());
//
//    double theta2 = atan2(goalPoint.y()-myPoint.y(), goalPoint.x()-myPoint.x());
//
//    double angle2 = 0.0;
//    if(-HALF_ROUND_DEG < theta2 && theta2<-M_PI/2){
//        theta2 = M_PI+theta2; //180�-the angle in the third section of the plane.
//        angle2 = M_PI/2 + theta2 - actual.getTheta();
//    } else {
//        angle2 = theta2 - (M_PI/2) - actual.getTheta();
//    }

//      NEW CODE
    double theta = atan2(pointToReach.y() - actual.y(),pointToReach.x() - actual.x());
    double angle2 = angularDistance(actual.theta()+M_PI_2, theta);

    return angle2;
}

#endif // UTILITIES_H
