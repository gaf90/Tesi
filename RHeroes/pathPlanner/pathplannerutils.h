#ifndef PATHPLANNERUTILS_H
#define PATHPLANNERUTILS_H

#include "RRT/rrtnode.h"
#include "slam/geometry/point.h"
#include "slam/map.h"

#define SAFETY_RADIUS 1.0

namespace PathPlanner{


inline double normalizeAngle(double angle)
{
    double toRet = wrapDeg(angle);
    if(toRet > fromDegreeToRadiants(HALF_ROUND_DEG))
        toRet = toRet - fromDegreeToRadiants(FULL_ROUND_DEG);
    else if (toRet < fromDegreeToRadiants(-HALF_ROUND_DEG))
        toRet = toRet + fromDegreeToRadiants(FULL_ROUND_DEG);

    return toRet;
}

inline bool computeOrientation(RRTNode *vicino, RRTNode *close, double constraint)
{
    double xNTemp=vicino->getX() - close->getX(), yNTemp = vicino->getY() - close->getY();
    double nearTheta = atan2(yNTemp,xNTemp);
    double steer = nearTheta-close->getTheta();
    steer = normalizeAngle(steer);
    vicino->setSteeringAngle(steer);
    double newTheta = close->getTheta()+steer;
    newTheta = normalizeAngle(newTheta);
    vicino->setTheta(newTheta);

    if(constraint == -1)
        return true;
    else{
        double computedSteer = fabs(steer);
        if(computedSteer > constraint){
            return false;
        } else {
            return true;
        }
    }
    return true;
}

inline bool validPoint(SLAM::Geometry::Point p, SLAM::Geometry::LineSegment ls)
{

    //the generated point is too close to the p1 vertex
    if(ls.p1().distance(p) < ROBOT_DIAG)
        return false;
    //the generated point is too close to the p2 vertex
    if(ls.p2().distance(p) < ROBOT_DIAG)
        return false;

    //The distance segment-point is lesser than the diagonal of the robot
    //and the projection of the point is inside the segment;
    double projection = ls.project1D(p);
    if(ls.distance(p) < ROBOT_DIAG && (projection>=0 && projection<=ls.length()))
        return false;

    return true;
}

inline bool validPoint(double x, double y, SLAM::Map *map)
{
    SLAM::Geometry::Point p(x, y);
    foreach(SLAM::Geometry::LineSegment *ls, map->walls()){
        if(!validPoint(p, *ls))
            return false;
    }
    //No constraint w.r.t. segments is violated
    return true;

}

inline bool inFreeSpace(SLAM::Geometry::LineSegment toTest, SLAM::Map *map)
{
    foreach(SLAM::Geometry::LineSegment *ls, map->walls()){
        if(ls->intersects(toTest))
            return false;
    }
    return true;
}

inline bool frontierFound(const QList<SLAM::Geometry::Frontier *> &frontiers, SLAM::Geometry::Point &posePoint)
{
    bool frontierFound = false;
    foreach (SLAM::Geometry::Frontier *frontier, frontiers){
        if(frontier->centroid().distance(posePoint) <= ROBOT_DIAG){
            frontierFound = true;
            break;
        }
    }
    return frontierFound;
}


}

#endif // PATHPLANNERUTILS_H
