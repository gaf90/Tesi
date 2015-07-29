#ifndef DOORLINESEGMENT_H
#define DOORLINESEGMENT_H
#include "slam/geometry/linesegment.h"
#include <QList>


namespace SemanticMapping{
class Room;

namespace Geometry{

class DoorLineSegment : public SLAM::Geometry::LineSegment
{
public:
    /**
     * Initialises a line segment with endpoints p1 and p2
     * @param p1 The first endpoint
     * @param p2 The second endpoint
     */
    DoorLineSegment(const SLAM::Geometry::Point &p1, const SLAM::Geometry::Point &p2);

    /**
     * Initialises a line segment with endpoints (x1,y1) and (x2,y2)
     * @param x1 The x coordinate of the first endpoint
     * @param y1 The y coordinate of the first endpoint
     * @param x2 The x coordinate of the second endpoint
     * @param y2 The y coordinate of the second endpoint
     */
    DoorLineSegment(double x1, double y1, double x2, double y2);

    /**
      * Returns a linesegment perpendicular to the door, with a fixed lenght and intersecting the
      * door in its centroid;
      * @returns A segment perpendicular to the door, passing from its centroid;
      */
    SLAM::Geometry::LineSegment getPSegment();

    /**
      * Returns a linesegment perpendicular to the door, with a given lenght and intersecting the
      * door in its centroid;
      * @param distance The lenght of the segment to be returned;
      * @returns A segment perpendicular to the door, passing from its centroid;
      */
    SLAM::Geometry::LineSegment getPSegment(double distance);

    void setFirstRoom(Room *firstRoom);
    void setSecondRoom(Room *secondRoom);
    bool isFirstOK();
    bool isSecondOK();
    bool isAllSet();

private:
    SemanticMapping::Room *first;
    SemanticMapping::Room *second;
};
}
}
#include "semanticMapping/room.h"

#endif // DOORLINESEGMENT_H
