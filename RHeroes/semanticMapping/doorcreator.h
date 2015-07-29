#ifndef DOORCREATOR_H
#define DOORCREATOR_H

#include "slam/map.h"
#include "slam/geometry/linesegment.h"
#include "semanticMapping/geometry/doorlinesegment.h"

namespace SemanticMapping {

namespace SemanticData {
class TopologicalMap;
}

class DoorCreator
{
public:
    /**
      * DoorCreator is a simpler version of SemanticHandler; It taks the map as input and
      * returns the list of the doors in the map as output;
      * @param inMap The map of the environment perceived by the robot;
      */
    DoorCreator(const SLAM::Map *inMap);

    /**
      * Destructor;
      */
    virtual ~DoorCreator();


    /**
      * @returns The list of all the doors;
      */
    QList<SemanticMapping::Geometry::DoorLineSegment> getDoors();

    friend class SemanticMapping::SemanticData::TopologicalMap;

private:
    typedef struct initWallStruct
    {
        double angle;
        QList<SLAM::Geometry::LineSegment> line;
        bool orizontal, vertical;
    } initWallStruct;
    typedef struct secondWallStruct
    {
        double M, Q, QM;
        QList<SLAM::Geometry::LineSegment> line;
        bool orizontal, vertical;
    } secondWallStruct;
    const SLAM::Map *map;
    QList<Geometry::DoorLineSegment> doors;
    void CreateWalls();
    void WallClusterHandler( QList<secondWallStruct> list );
};
}

#endif // DOORCREATOR_H
