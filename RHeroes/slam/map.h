/*
 * map.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef MAP_H_
#define MAP_H_

#include <QList>
#include <QHash>
#include "data/serializable.h"
#include "geometry/frontier.h"
#include "pathnode.h"
#include <QRect>

namespace SLAM {
namespace Legacy {
class OdometryOnlySLAM;
} /* namespace Legacy */

typedef enum {
    False = 0,
    True = 1,
    Maybe = 2,
    NotInPrefetch = 3
} boolplus;

class Map : public Data::Serializable
{
public:
    Map();
    Map(const Map &map);
    virtual ~Map();

    const QList<Geometry::LineSegment> getWalls() const __attribute__((deprecated));
    const QList<Geometry::Frontier> getFrontiers() const __attribute__((deprecated));
    const QList<TimedPose> getRobotPath(uint id) const __attribute__((deprecated));
    const TimedPose &getRobotPose(uint id) const __attribute__((deprecated));



    const QList<Geometry::LineSegment *> &walls() const { return qwalls; }
    const QList<Geometry::LineSegment *> &obstacles() const { return qobstacles; }
    const QList<Geometry::Frontier *> &frontiers() const { return qfrontiers; }

    QList<Geometry::LineSegment *> &walls() { return qwalls; }
    QList<Geometry::LineSegment *> &obstacles() { return qobstacles; }
    QList<Geometry::Frontier *> &frontiers() { return qfrontiers; }

    const QList<PathNode *> &robotPath(uint id) const;
    const PathNode *lastRobotPose(uint id) const;
    const PathNode *firstRobotPose(uint id) const;

    void addWall(const Geometry::LineSegment &s);
    void addObstacle(const Geometry::LineSegment &s);
    void addFrontier(const Geometry::Frontier &f);
    void addPose(uint id, const TimedPose &pose);

    void prefetch(const Geometry::Point &center, double radius);
    void clearPrefetch();

    boolplus isReachable(
            const Geometry::Point &from, const Geometry::Point &to, double radius) const;
    boolplus isReachableWithPrefetch(
            const Geometry::Point &from, const Geometry::Point &to, double radius) const;

    QList<uint> knownRobots() const;

    double getMinX() const;
    double getMinY() const;
    double getMaxX() const;
    double getMaxY() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    Map &operator=(const Map &map);

    friend class Legacy::OdometryOnlySLAM; // TODO: temporaneo

private:
    template <typename T>
    bool listIntersects(
            const QList<T *> &l, const Geometry::Point &from,
            const Geometry::Point &to, double radius) const;
    void freeContent();


    QList<Geometry::LineSegment *> qwalls, prefetchWalls;
    QList<Geometry::LineSegment *> qobstacles, prefetchObstacles;
    QList<Geometry::Frontier *> qfrontiers, prefetchFrontiers;

    QHash<uint, QList<PathNode *> > robotPaths;

    double minx, miny, maxx, maxy;
    Geometry::Point prefetchCenter;
    double prefetchRadius;
    bool owner;
};

inline void Map::addWall(const Geometry::LineSegment &s)
{
    qwalls.append(new Geometry::LineSegment(s));
    minx = min(minx, s.p1().x(), s.p2().x());
    miny = min(miny, s.p1().y(), s.p2().y());
    maxx = max(maxx, s.p1().x(), s.p2().x());
    maxy = max(maxy, s.p1().y(), s.p2().y());
}
inline void Map::addObstacle(const Geometry::LineSegment &s)
{
    qobstacles.append(new Geometry::LineSegment(s));
}


inline void Map::addFrontier(const Geometry::Frontier &f)
{
    qfrontiers.append(new Geometry::Frontier(f));
}

inline void Map::addPose(uint id, const TimedPose &pose)
{
    PathNode *n = new PathNode(pose);
    if(robotPaths.contains(id)) {
        n->setPrevious(robotPaths[id].last());
        robotPaths[id].last()->setNext(n);
    } else {
        robotPaths.insert(id, QList<PathNode *>());
    }
    robotPaths[id].append(n);
}


} /* namespace SLAM */

#endif /* MAP_H_ */
