#ifndef VICTIM_H
#define VICTIM_H

#include "data/pose.h"
#include <QImage>

#define VM_EQUALITY_DISTANCE_TH 5
#define VM_EQUALITY_ANGLE_TH 90

namespace BaseStation{

/**
* This class represents a Victim. It contains its pose, images representing it, and some useful
* operator that makes possible to "compare" victims. The class also contains also some information
* about discovering robot and assigned robot. Assigned robot may vary over time, but discovering one
* (the first one that transmitted the message to the baseStation showing the victim) won't.
*/
class Victim
{
public:
    //! Constructor for victims retrieved by the victim detection module.
    Victim(uint discoveredBy = 0, uint id = 0, Data::Pose *p = 0);

    //! Constructor for victims retrieved fully manually by the operator.
    Victim(uint id, Data::Pose *p = 0);

    //! Destroys the Victim object
    virtual ~Victim();

    //! assigns the victim to the robot with id robotID.
    void assignToRobot(uint robot);

    //! sets the time that the assignedRobot needs to reach the victim. Used only for UI purposes.
    void setTimeToReach(int time);

    //! adds an image to the victim's album. Can be used to manually compare victims
    void addImage(const QImage &img);

    //! This method can be used to compare victims and avoid multiple detections
    bool equals(Victim v);

    bool isConfirmed();

    void confirm();

    void setPosition(const Data::Pose &p);

    //getters
    uint getVictimID();
    uint getDiscoverererID();
    uint getAssignedRobotID();
    QString printAssignedRobotName();
    const Data::Pose &getPosition() const;
    const QList<QImage> &getImages() const;
    const QImage &getImage(int i) const;
    int getTimeToBeReached();
    QString printTimeToBeReached();

private:

    uint victimID;
    uint discoveringRobot;
    uint assignedRobot;
    Data::Pose *position;
    QList<QImage> *images;
    /**
    * This is the estimated time that the assignedRobot will take to reach the victim if
    * autonomously drived.
    */
    int timeToReach;
    bool confirmed;
};

}

#endif // VICTIM_H
