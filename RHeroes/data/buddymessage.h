#ifndef BUDDYMESSAGE_H
#define BUDDYMESSAGE_H

#include "message.h"
#include "serializable.h"
#include "data/wheelmessage.h"
#include "data/cameradata.h"
#include "data/usarmessage.h"
#include "data/victimmessage.h"
#include "data/victimdelectionmessage.h"
#include "data/mapmessage.h"
#include "data/semanticmapinfomessage.h"
#include "data/highlevelcommand.h"
#include "data/infomessage.h"
#include "data/askbidsmessage.h"
#include "data/sendbidsmessage.h"
#include "data/assignitemmessage.h"
#include "data/waypointcommand.h"
#include "data/moduleactivationmessage.h"
#include "data/ackmessage.h"
#include "data/robotvictimcouplingmessage.h"
#include "data/victimconfirmationmessage.h"
#include "data/distancevectormessage.h"
#include "data/destinationmessage.h"
#include "data/victimdetectionconfmessage.h"
#include "data/airdrivemessage.h"
#include "data/errornotificationmessage.h"
#include <typeinfo>
#include <QDataStream>

namespace Data {

class BuddyMessage : public Message, public Serializable
{
public:
    /**
     * Specifies the type of information conveyed by a BuddyMessage.
     * The information may be either atomic or requiring additional data
     * in the form of a Serializable object specified between brackets.
     */
    enum Content {

        //! No information, for default constructor
        Empty,

        //! Request wheel motion (WheelMessage)
        WheelMotion,

        //! Request air motion (AirMessage)
        AirMotion,

        //! Request to enable camera frame streaming from the target robot to
        //! this robot/base station
        EnableCameraStreaming,

        //! Request to disable camera frame streaming from the target robot to
        //! this robot/base station
        DisableCameraStreaming,

        //! Report to the target robot a camera frame (CameraData)
        CameraFrame,

        //! Request execution of raw USARSim command (USARMessage)
        RawUSARCommand,

        //! Sends information about a victim detected to the base station
        VictimInformation,

        //! Sends information about a Map, with victims and robots positions and timestamp
        MapInformation,

        //! Sends semantic information about an area, identified by a center point or by a
        //! set of points representing the area's perimeter
        SemanticInformation,

        //! Sends high levels commands to single robots
        HighCommand,

        //! Sends single robots informations and respective requests
        InformationTransfer,

        //! Asks for bids on a set of frontiers
        AskBidsOnFrontiers,

        //! Sends bids on a set of frontiers
        SendBidsOnFrontiers,

        //! Assigns frontier to a robot
        FrontierAssignment,

        //! Asks for bids on a set of victims
        AskBidsOnVictims,

        //! Sends bids on a set of victims
        SendBidsOnVictims,

        //! Assigns a victim to a robot
        VictimAssignment,

        //! Deletes the victim from the system
        VictimDeletion,

        //! Confirms the existence of a victim to a robot
        VictimConfirmation,

        //! Assigns a list of waypoints to follow
        WaypointMessage,

        //! Activates/disactivates a module of a robot
        ModuleActivation,

        //! Acknowledgment of message received
        AcknowledgmentMessage,

        //! Sending information about association robot-victim and estimated time to reach pose
        RobotVictimCoupling,

        //! Distance Vector information for routing purposes
        DistanceVector,

        //! Notify to the base station the point the robot is pointing
        DestinationNotificationMessage,

        //! Sends the parameters of the victim detection module to single robots
        VictimDetectionConfiguration,

        ErrorNotification

    };

    BuddyMessage();
    BuddyMessage(const BuddyMessage &buddy);
    BuddyMessage(const QString &src, const QString &dest,
                 BuddyMessage::Content content, Serializable *raw = NULL);
    BuddyMessage(const QString &src, const QString &dest, const WheelMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const AirDriveMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const CameraData *data);
    BuddyMessage(const QString &src, const QString &dest, const USARMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const AskBidsMessage<SLAM::Geometry::Frontier> *data);
    BuddyMessage(const QString &src, const QString &dest, const SendBidsMessage *data, Content content);//needed because due to the not templatized message it is not possible to distinguish the different auctions
    BuddyMessage(const QString &src, const QString &dest, const AssignItemMessage<SLAM::Geometry::Frontier> *data);
    BuddyMessage(const QString &src, const QString &dest, const AskBidsMessage<uint> *data);
    BuddyMessage(const QString &src, const QString &dest, const AssignItemMessage<uint> *data);
    BuddyMessage(const QString &src, const QString &dest, const AckMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const InfoMessage *data);
    BuddyMessage(const QString &src, const VictimMessage *data);
    BuddyMessage(const QString &src, const RobotVictimCouplingMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const VictimDelectionMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const VictimConfirmationMessage *data);
    BuddyMessage(const QString &src, const QString &dest, const DistanceVectorMessage *data);
    BuddyMessage(const QString &src, const DestinationMessage *data);
    BuddyMessage(const QString &dest, const VictimDetectionConfMessage *data);

    virtual ~BuddyMessage();

    const Serializable *getRawContent() const;
    const CameraData *getCameraData() const;
    const USARMessage *getUSARMessage() const;
    const AskBidsMessage<SLAM::Geometry::Frontier> *getAskBidsOnFrontiersMessage() const; //TODO, probably not used, check
    const AckMessage *getAckMessage() const; //TODO, probably not used, check
    /*const SendBidsMessage *getSendBidsOnFrontiersMessage() const;
    const AssignItemMessage<SLAM::Geometry::Frontier *> *getAssignItemOnFrontierMessage() const;*/
//--------Base station communication messages
    const WheelMessage *getWheelMessage() const;
    const AirDriveMessage *getAirDriveMessage() const;
    const SemanticMapInfoMessage *getSemanticMapInfoMessage() const;
    const HighLevelCommand *getHighLevelCommand() const;
    const InfoMessage *getInfoMessage() const;
    const MapMessage *getMapMessage() const;
    const ModuleActivationMessage *getModuleActivationMessage() const;
    const VictimMessage *getVictimMessage() const;
    const WaypointCommand *getWaypointCommand() const;
    const DistanceVectorMessage *getDistanceVector() const;

    template <typename T>
    const T *get() const;


    BuddyMessage::Content getContent() const;

    const QString &getDestination() const;
    const QString &getSource() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    QString source, destination;
    Content content;
    const Serializable *raw;
    bool hasOwnership;
};

template <typename T>
const T *BuddyMessage::get() const
{
    const T *ret = dynamic_cast<const T *>(raw);
    return ret;
}

LoggerStream &operator<<(LoggerStream &stream, BuddyMessage::Content c);

}

#endif // BUDDYMESSAGE_H
