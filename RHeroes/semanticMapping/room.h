#ifndef ROOM_H
#define ROOM_H
#include "slam/geometry/point.h"
#include "slam/map.h"
#include "semanticMapping/SemanticDef.h"
#include "semanticMapping/geometry/doorlinesegment.h"
#include <QList>

namespace SemanticMapping{
class Room
{
public:
    /**
      * Initialize a nil room;
      */
    Room();

    /**
      * Initialize a room, given a list of segments belonging to the room and a core point;
      * @param prevRoom List of walls that constitutes the room;
      * @param init The initial point; the walls are all the linesegments of the map directly visible from init;
      */
    Room(QList<SLAM::Geometry::LineSegment> &prevRoom, SLAM::Geometry::Point &init);

    /**
      * Initialize a room given the core point of the room;
      * @param init The initial point; all the walls of the room should be directly visible from init;
      */
    Room(SLAM::Geometry::Point &init);

    /**
      * Copy constructor
      */
    Room(const Room &r);

    /**
      * Destructor;
      */
    virtual ~Room();

    /**
      * Add a new segment to the room;
      * @param newseg The segment added to the room;
      */
    void addSegment(SLAM::Geometry::LineSegment newseg);

    /**
      * Add a new door to the room;
      * @param newdoor The door added to the room;
      */
    void addDoor(SemanticMapping::Geometry::DoorLineSegment newdoor);

    /**
      * Add a new frontier to the room;
      * @param newfrontier The frontier added to the room;
      */
    void addFrontier(SLAM::Geometry::Frontier newfrontier);

    /**
      * Merge operation for two different room objects that are the same room in the map;
      * @param r The room that will be merged in the initial one;
      */
    void addOtherRoom(const Room &r);

    /**
      * Remove a segment from the walls of the room, if present;
      * @param wrong The segment to be removed;
      */
    void removeSegment(SLAM::Geometry::LineSegment wrong);

    /**
      * @returns The list of the linesegment of the room, the walls of the room;
      */
    const QList<SLAM::Geometry::LineSegment> &getRoom();

    /**
      * @returns The list of the doors of the room;
      */
    const QList<SemanticMapping::Geometry::DoorLineSegment> &getDoors();

    /**
      * @returns The list of the frontiers of the map belonging to the room;
      */
    const QList<SLAM::Geometry::Frontier> &getFrontiers();

    /**
      * Checks if a given point is inside or outside the room;
      * @param P The point to be checked;
      * @returns if the point is inside the room;
      */
    bool isInRoom( SLAM::Geometry::Point &p);

    /**
      * @returns The label of the room;
      */
    RoomCat getInfo();

    /**
      * @returns The area of the room;
      */
    double getArea();

    /**
      * @returns The number of doors of the room;
      */
    int getGrade();

    /**
      * @returns the lenght of the main dimension of the room;
      */
    double getCorridorLenght();

    /**
      * @returns the core point of the room, the one given at initialization;
      */
    SLAM::Geometry::Point getCorePoint();

    /**
      * @returns four points that represents a quadrilateral in which the room is inscribed;
      */
    QList<SLAM::Geometry::Point> *getBorderPoint();

    /**
      * @returns the central point of the room;
      */
    SLAM::Geometry::Point *getCentroid();

    /**
      * @returns checks if the room is already processed or not;
      */
    bool roomHandled();

    /**
      * Closes the initialization phase of the room and calculate all the informations;
      * @returns a flag representig the correct end of the function;
      */
    bool handleRoom();

private:
    bool finished;
    QList<SLAM::Geometry::LineSegment> walls;
    QList<Geometry::DoorLineSegment> doors;
    QList<SLAM::Geometry::Frontier> frontiers;
    double area;
    double corridorLenght;
    SLAM::Geometry::Point n_o, n_e, s_o, s_e, corepoint, centroid;
    RoomCat roomlabel;
    void calcArea();
    void classifyRoom(double d1, double d2);
    typedef struct initWallStruct
    {
        double angle;
        QList<SLAM::Geometry::LineSegment> line;
        bool orizontal, vertical;
        double lenght;
    } initWallStruct;
    typedef struct secondWallStruct
    {
        double M, Q, QM;
        QList<SLAM::Geometry::LineSegment> line;
        bool orizontal, vertical;

    } secondWallStruct;
    typedef struct thirdWallStruct
    {
        double angle;
        double lenght;
    } thirdWallStruct;
    thirdWallStruct roomClassifierHelper( QList<secondWallStruct> list );
};
}
#endif // ROOM_H
