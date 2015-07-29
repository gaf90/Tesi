/*
 * map.cpp
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#include "map.h"
#include "slam/geometry/quadrilateral.h"
#include "slam/utilities.h"

namespace SLAM {

using namespace Geometry;

Map::Map() :
    robotPaths(), minx(INFINITY), miny(INFINITY), maxx(-INFINITY), maxy(-INFINITY),
    prefetchCenter(INFINITY, INFINITY), prefetchRadius(0), owner(true)
{
}

Map::Map(const Map &map) :
    Serializable(), robotPaths()
{
    *this = map;
}

Map::~Map()
{
    freeContent();
}

const QList<LineSegment> Map::getWalls() const
{
    QList<LineSegment> newWalls;
    fforeach(LineSegment *s, qwalls) {
        newWalls << *s;
    }
    return newWalls;
}

const QList<Frontier> Map::getFrontiers() const
{
    QList<Frontier> newFrontiers;
    fforeach(Frontier *f, qfrontiers) {
        newFrontiers << *f;
    }
    return newFrontiers;
}

const TimedPose &Map::getRobotPose(uint id) const
{
    return *(robotPaths[id].last());
}

const QList<TimedPose> Map::getRobotPath(uint id) const
{
    QList<TimedPose> poses;
    fforeach(PathNode *p, robotPaths[id]) {
        poses << *p;
    }
    return poses;
}

const QList<PathNode *> &Map::robotPath(uint id) const
{
    QHash<uint, QList<PathNode *> > &m = const_cast<QHash<uint, QList<PathNode *> > &>(robotPaths);
    QList<PathNode *> &l = m[id];
    return l;
}

const PathNode *Map::lastRobotPose(uint id) const
{
    return robotPath(id).last();
}

const PathNode *Map::firstRobotPose(uint id) const
{
    return robotPath(id).first();
}

QList<uint> Map::knownRobots() const
{
    return robotPaths.keys();
}

void Map::freeContent()
{
    if(owner) {
        fforeach(LineSegment *s, qwalls) {
            delete s;
        }
        fforeach(LineSegment *s, qobstacles) {
            delete s;
        }
        fforeach(Frontier *f, qfrontiers) {
            delete f;
        }
        for(QHash<uint, QList<PathNode *> >::iterator it = robotPaths.begin();
            it != robotPaths.end(); it++) {
            QList<PathNode *> &nodes = it.value();
            fforeach(PathNode *n, nodes) {
                delete n;
            }
        }
    }
}

void Map::serializeTo(QDataStream &stream) const
{
    /* Serialize sizes */
    stream << qwalls.size() << qobstacles.size() << qfrontiers.size() << robotPaths.size();

    /* Serialize all walls */
    fforeach(LineSegment *s, qwalls) {
        stream << *s;
    }

    /* Serialize all obstacles */
    fforeach(LineSegment *s, qobstacles) {
        stream << *s;
    }

    /* Serialize all frontiers */
    fforeach(Frontier *f, qfrontiers) {
        stream << *f; /* Doesn't serialize PathNode visibility information */
    }

    /* Serialize all robot paths */
    for(QHash<uint, QList<PathNode *> >::const_iterator it = robotPaths.begin();
        it != robotPaths.end(); it++) {
        /* For each robot serialize the id, the size of the path and the actual path */
        stream << it.key() << it.value().size();
        const QList<PathNode *> &nodes = it.value();
        fforeach(PathNode *n, nodes) {
            stream << *n;
#if 0
            /* Serialize the pose information of the path node and the sizes */
            stream << /*n->visibleWalls().size() <<*/ n->visibleFrontiers().size() <<
                      n->visibilities().size();

            //foreach(LineSegment *s, n->visibleWalls()) {
            //    stream << qwalls.indexOf(s); /* Serialize the indices od the walls */
            //}
            fforeach(Frontier *f, n->visibleFrontiers()) {
                stream << qfrontiers.indexOf(f); /* Serialize the indices of the frontiers */
            }
            fforeach(VisibilityTriangle *v, n->visibilities()) {
                stream << *v; /* Serialize the visibility triangles */
            }
#endif
        }
    }
    /* Serialize the map bounds */
    stream << minx << miny << maxx << maxy;
}
void Map::deserializeFrom(QDataStream &stream)
{
    int wsize, osize, fsize, rsize, psize;
    stream >> wsize >> osize >> fsize >> rsize;
    owner = true;
    for(int i = 0; i < wsize; i++) {
        LineSegment *l = new LineSegment();
        stream >> *l;
        qwalls << l;
    }
    for(int i = 0; i < osize; i++) {
        LineSegment *l = new LineSegment();
        stream >> *l;
        qobstacles << l;
    }
    for(int i = 0; i < fsize; i++) {
        Frontier *f = new Frontier();
        stream >> *f;
        qfrontiers << f;
    }
    for(int i = 0; i < rsize; i++) {
        uint rid;
        stream >> rid >> psize;
        QList<PathNode *> path;
        for(int j = 0; j < psize; j++) {
            PathNode *n = new PathNode();
            stream >> *n;
#if 0
            int idx, /*wsize,*/ fsize, vsize;
            stream /*>> wsize*/ >> fsize >> vsize;
            /*for(int k = 0; k < wsize; k++) {
                stream >> idx;
                n->visibleWalls().append(qwalls[idx]);
            }*/
            for(int k = 0; k < fsize; k++) {
                stream >> idx;
                n->visibleFrontiers().append(qfrontiers[idx]);
                qfrontiers[idx]->setVisibility(n);
            }
            for(int k = 0; k < vsize; k++) {
                VisibilityTriangle *v = new VisibilityTriangle();
                stream >> *v;
                n->visibilities().append(v);
                v->setVisibility(n);
            }
#endif
            path << n;
        }
        robotPaths.insert(rid, path);
    }
    stream >> minx >> miny >> maxx >> maxy;
}

double Map::getMinX() const
{
    return minx;
}

double Map::getMinY() const
{
    return miny;
}

double Map::getMaxX() const
{
    return maxx;
}

double Map::getMaxY() const
{
    return maxy;
}

void Map::prefetch(const Point &center, double radius)
{
    prefetchRadius = radius;
    prefetchCenter = center;
    fforeach(LineSegment *l, qwalls) {
        if(l->intersectsCircle(center, radius)) {
            prefetchWalls.append(l);
        }
    }
    fforeach(LineSegment *l, qobstacles) {
        if(l->intersectsCircle(center, radius)) {
            prefetchObstacles.append(l);
        }
    }
    fforeach(Frontier *f, qfrontiers) {
        if(f->intersectsCircle(center, radius)) {
            prefetchFrontiers.append(f);
        }
    }
}

void Map::clearPrefetch()
{
    prefetchWalls.clear();
    prefetchObstacles.clear();
    prefetchFrontiers.clear();
    prefetchCenter = Point(INFINITY, INFINITY);
    prefetchRadius = 0;
}

Map &Map::operator=(const Map &map)
{
    freeContent();

    minx = map.minx;
    miny = map.miny;
    maxx = map.maxx;
    maxy = map.maxy;
    prefetchCenter = Point(INFINITY, INFINITY);
    prefetchRadius = 0;
    owner = true;

    qwalls.clear();
    qfrontiers.clear();
    robotPaths.clear();

    fforeach(const LineSegment *s, map.qwalls) {
        qwalls.append(new LineSegment(*s));
    }
    fforeach(const LineSegment *s, map.qobstacles) {
        qobstacles.append(new LineSegment(*s));
    }
    fforeach(const Frontier *f, map.qfrontiers) {
        qfrontiers.append(new Frontier(*f));
    }
    for(QHash<uint, QList<PathNode *> >::const_iterator it = map.robotPaths.begin();
                it != map.robotPaths.end(); it++) {
        robotPaths.insert(it.key(), QList<PathNode *>());
        const QList<PathNode *> &l = it.value();
        if(l.size() > 0) {
            QList<PathNode *> &addTo = robotPaths[it.key()];
            PathNode *n = new PathNode(*l.first());
            addTo.append(n);
            while(n->next() != NULL) {
                n->setNext(new PathNode(*n->next()));
                n = n->next();
                addTo.append(n);
            }
        }
    }
    return *this;
}

template <typename T>
bool Map::listIntersects(
        const QList<T *> &l, const Geometry::Point &from,
        const Point &to, double radius) const
{
    /*
        Build two line segments, each parallel to the one connecting "from" to "to" and
        perpendicularly distant by a "radius" amount
     */

    //PRM
    if(radius==0){
        LineSegment seg(from,to);
        foreach(const T* s, l){
            if(s->intersects(seg)){
                return true;
            }
        }
        return false;
    }
    //

    const double theta = LineSegment(from, to).angle();
    const double dx = radius * std::sin(theta), dy = radius * std::cos(theta);
    LineSegment l1(from.x() + dx, from.y() - dy, to.x() + dx, to.y() - dy);
    LineSegment l2(from.x() - dx, from.y() + dy, to.x() - dx, to.y() + dy);

    fforeach(const T *s, l) {
        /* Try to intersect with the two circles centered at "from" and "to" */
        if(s->intersectsCircle(from, radius)) return true;
        if(s->intersectsCircle(to,   radius)) return true;


        /* Try to intersect with the two line segments */
        if(s->intersects(l1)) return true;
        if(s->intersects(l2)) return true;

        /* Build a quadrilateral having as edges the two line segments */
        Quadrilateral q(l1.p1(), l1.p2(), l2.p2(), l2.p1());

        /* Check if one of the end points is contained in the quadrilateral */
        if(q.contains(s->p1()) || q.contains(s->p2())) return true;
    }

    return false;
}

boolplus Map::isReachable(const Point &from, const Point &to, double radius) const
{
    if(listIntersects(qwalls, from, to, radius) || listIntersects(qobstacles, from, to, radius))
        return False;
    else if(listIntersects(qfrontiers, from, to, radius))
        return Maybe;
    else
        return True;
}

boolplus Map::isReachableWithPrefetch(const Point &from, const Point &to, double radius) const
{
    if(prefetchCenter.distance(from) > prefetchRadius - radius ||
            prefetchCenter.distance(to) > prefetchRadius - radius)
        return NotInPrefetch;
    else if(listIntersects(prefetchWalls, from, to, radius) || listIntersects(prefetchWalls, from, to, radius))
        return False;
    else if(listIntersects(prefetchFrontiers, from, to, radius))
        return Maybe;
    else
        return True;
}

} /* namespace SLAM */
