#ifndef SEMANTICHANDLER_H
#define SEMANTICHANDLER_H
#include "slam/map.h"
#include "room.h"
#include "slam/geometry/linesegment.h"
#include "semanticMapping/geometry/doorlinesegment.h"

namespace SemanticMapping {

namespace SemanticData {
class TopologicalMap;
}

class SemanticHandler
{
public:
    /**
      * Initialize the handler, which creates the list of the room and the doors;
      * @param inMap The map of the environment perceived by the robot;
      */
    SemanticHandler(const SLAM::Map *inMap);

    /**
      * Destructor;
      */
    virtual ~SemanticHandler();

    /**
      * Find the room where the given point is; returns the semantic informations about that room;
      * @param the point to be checked;
      * @returns the infos of the room where the given point is;
      *          if the point is outside of all the rooms it returns NULL;
      */
    semanticInfos *getSemanticInfos(SLAM::Geometry::Point point);

    /**
      * @returns The list of all the rooms;
      */
    QList<Room> getRooms();

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
    QList<Room> rooms;
    QList<Geometry::DoorLineSegment> doors;
    void CreateWalls();
    void CreateRooms();
    void CreateSingleRoom(SLAM::Geometry::Point p);
    Room* getRoomPoint(SLAM::Geometry::Point &point);
    void WallClusterHandler( QList<secondWallStruct> list );
    void MergeRooms();
    void RemoveRooms();
    void ProcessRooms();

};
}
#endif // SEMANTICHANDLER_H
