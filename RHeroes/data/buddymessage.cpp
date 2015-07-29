#include "buddymessage.h"

namespace Data {

BuddyMessage::BuddyMessage() :
    source(""), destination(""), content(BuddyMessage::Empty), raw(NULL),
    hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        BuddyMessage::Content content, Serializable *raw) :
    source(src), destination(dest), content(content), raw(raw),
    hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const CameraData *data) :
    source(src), destination(dest), content(BuddyMessage::CameraFrame),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const WheelMessage *data) :
    source(src), destination(dest), content(BuddyMessage::WheelMotion),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const QString &dest,
                           const AirDriveMessage *data):
    source(src), destination(dest), content(BuddyMessage::AirMotion),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const USARMessage *data) :
    source(src), destination(dest), content(BuddyMessage::RawUSARCommand),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const AskBidsMessage<SLAM::Geometry::Frontier> *data) :
    source(src), destination(dest), content(BuddyMessage::AskBidsOnFrontiers),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const SendBidsMessage *data, Content aContent) :
    source(src), destination(dest), content(aContent),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const AssignItemMessage<SLAM::Geometry::Frontier> *data) :
    source(src), destination(dest), content(BuddyMessage::FrontierAssignment),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const AskBidsMessage<uint> *data) :
    source(src), destination(dest), content(BuddyMessage::AskBidsOnVictims),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const AssignItemMessage<uint> *data) :
    source(src), destination(dest), content(BuddyMessage::VictimAssignment),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest, const VictimDelectionMessage *data):
    source(src), destination(dest), content(BuddyMessage::VictimDeletion), raw(data),
    hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const QString &dest,
                           const VictimConfirmationMessage *data) :
    source(src), destination(dest), content(BuddyMessage::VictimConfirmation), raw(data),
    hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const QString &dest, const DistanceVectorMessage *data):
    source(src), destination(dest), content(BuddyMessage::DistanceVector),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const BuddyMessage &buddy) :
    Message(), Serializable()
{
    QByteArray buffer;
    QDataStream stream(&buffer, QIODevice::ReadWrite);
    stream << buddy;
    stream.device()->seek(0);
    stream >> *this;
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const AckMessage *data) :
    source(src), destination(dest), content(BuddyMessage::AcknowledgmentMessage),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(
        const QString &src, const QString &dest,
        const InfoMessage *data) :
    source(src), destination(dest), content(BuddyMessage::InformationTransfer),
    raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const VictimMessage *data)
    : source(src), destination(robotNameFromIndex(BASE_STATION_ID)), content(BuddyMessage::VictimInformation), raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const RobotVictimCouplingMessage *data)
    : source(src), destination(robotNameFromIndex(BASE_STATION_ID)), content(BuddyMessage::RobotVictimCoupling), raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &src, const DestinationMessage *data)
    : source(src), destination(robotNameFromIndex(BASE_STATION_ID)), content(BuddyMessage::DestinationNotificationMessage), raw(data), hasOwnership(false)
{
}

BuddyMessage::BuddyMessage(const QString &dest, const VictimDetectionConfMessage *data)
    : source(robotNameFromIndex(BASE_STATION_ID)), destination(dest), content(BuddyMessage::VictimDetectionConfiguration), raw(data), hasOwnership(false)
{
}

BuddyMessage::~BuddyMessage()
{
    if(hasOwnership)
        delete raw;
}

const Serializable *BuddyMessage::getRawContent() const
{
    return raw;
}

const WheelMessage *BuddyMessage::getWheelMessage() const
{
    if(content != BuddyMessage::WheelMotion) {
        return NULL;
    } else {
        return (const WheelMessage *) raw;
    }
}

const AirDriveMessage *BuddyMessage::getAirDriveMessage() const
{
    if(content != BuddyMessage::AirMotion) {
        return NULL;
    } else {
        return (const AirDriveMessage *) raw;
    }
}

const AskBidsMessage<SLAM::Geometry::Frontier> *BuddyMessage::getAskBidsOnFrontiersMessage() const
{
    if(content != BuddyMessage::AskBidsOnFrontiers) {
        return NULL;
    } else {
        return (const AskBidsMessage<SLAM::Geometry::Frontier> *) raw;
    }
}

const AckMessage * BuddyMessage::getAckMessage() const
{
    if(content != BuddyMessage::AcknowledgmentMessage) {
        return NULL;
    } else {
        return (const AckMessage *) raw;
    }
}

/*
const Data::SendBidsMessage * BuddyMessage::getSendBidsOnFrontiersMessage() const
{
    if(content != BuddyMessage::SendBids) {
        return NULL;
    } else {
        return (const SendBidsMessage *) raw;
    }
}

const Data::AssignItemMessage<SLAM::Geometry::Frontier *> * BuddyMessage::getAssignItemOnFrontierMessage() const
{
    if(content != BuddyMessage::FrontierAssignment) {
        return NULL;
    } else {
        return (const AssignItemMessage<SLAM::Geometry::Frontier *> *) raw;
    }
}*/


const CameraData *BuddyMessage::getCameraData() const
{
    if(content != BuddyMessage::CameraFrame) {
        return NULL;
    } else {
        return (const CameraData *) raw;
    }
}

const USARMessage *BuddyMessage::getUSARMessage() const
{
    if(content != BuddyMessage::RawUSARCommand) {
        return NULL;
    } else {
        return (const USARMessage *) raw;
    }
}

const SemanticMapInfoMessage * BuddyMessage::getSemanticMapInfoMessage() const
{
    if(content != BuddyMessage::SemanticInformation) {
        return NULL;
    } else {
        return (const SemanticMapInfoMessage *) raw;
    }
}

const HighLevelCommand * BuddyMessage::getHighLevelCommand() const
{
    if(content != BuddyMessage::HighCommand) {
        return NULL;
    } else {
        return (const HighLevelCommand *) raw;
    }
}

const InfoMessage * BuddyMessage::getInfoMessage() const
{
    if(content != BuddyMessage::InformationTransfer) {
        return NULL;
    } else {
        return (const InfoMessage *) raw;
    }
}

const MapMessage * BuddyMessage::getMapMessage() const
{
    if(content != BuddyMessage::MapInformation) {
        return NULL;
    } else {
        return (const MapMessage *) raw;
    }
}

const ModuleActivationMessage * BuddyMessage::getModuleActivationMessage() const
{
    if(content != BuddyMessage::ModuleActivation) {
        return NULL;
    } else {
        return (const ModuleActivationMessage *) raw;
    }
}

const VictimMessage * BuddyMessage::getVictimMessage() const
{
    if(content != BuddyMessage::VictimInformation) {
        return NULL;
    } else {
        return (const VictimMessage *) raw;
    }
}

const WaypointCommand * BuddyMessage::getWaypointCommand() const
{
    if(content != BuddyMessage::WaypointMessage) {
        return NULL;
    } else {
        return (const WaypointCommand *) raw;
    }
}

const DistanceVectorMessage *BuddyMessage::getDistanceVector() const
{
    if(content != BuddyMessage::DistanceVector) {
        return NULL;
    } else {
        return (const DistanceVectorMessage *) raw;
    }
}

BuddyMessage::Content BuddyMessage::getContent() const
{
    return content;
}

const QString &BuddyMessage::getDestination() const
{
    return destination;
}

const QString &BuddyMessage::getSource() const
{
    return source;
}

void BuddyMessage::serializeTo(QDataStream &stream) const
{
    quint8 content_id = content;
    stream << content_id << source << destination;
    if(content == BuddyMessage::CameraFrame ||
            content == BuddyMessage::WheelMotion ||
            content == BuddyMessage::AirMotion ||
            content == BuddyMessage::RawUSARCommand ||
            content == BuddyMessage::VictimInformation ||
            content == BuddyMessage::MapInformation ||
            content == BuddyMessage::SemanticInformation||
            content == BuddyMessage::HighCommand ||
            content == BuddyMessage::InformationTransfer ||
            content == BuddyMessage::AskBidsOnFrontiers ||
            content == BuddyMessage::SendBidsOnFrontiers ||
            content == BuddyMessage::FrontierAssignment ||
            content == BuddyMessage::AskBidsOnVictims ||
            content == BuddyMessage::SendBidsOnVictims ||
            content == BuddyMessage::VictimAssignment ||
            content == BuddyMessage::WaypointMessage ||
            content == BuddyMessage::ModuleActivation ||
            content == BuddyMessage::AcknowledgmentMessage ||
            content == BuddyMessage::VictimConfirmation ||
            content == BuddyMessage::VictimDeletion ||
            content == BuddyMessage::RobotVictimCoupling ||
            content == BuddyMessage::DistanceVector ||
            content == BuddyMessage::DestinationNotificationMessage ||
            content == BuddyMessage::VictimDetectionConfiguration ||
            content == BuddyMessage::ErrorNotification) {
        stream << raw;
    }
}

void BuddyMessage::deserializeFrom(QDataStream &stream)
{
    quint8 content_id;
    stream >> content_id >> source >> destination;

    content = (BuddyMessage::Content) content_id;

    Serializable *obj;

    switch(content) {
    case BuddyMessage::CameraFrame:
        obj = new CameraData();
        break;
    case BuddyMessage::WheelMotion:
        obj = new WheelMessage();
        break;
    case BuddyMessage::AirMotion:
        obj = new AirDriveMessage();
        break;
    case BuddyMessage::RawUSARCommand:
        obj = new USARMessage();
        break;
    case BuddyMessage::VictimInformation:
        obj = new VictimMessage();
        break;
    case BuddyMessage::MapInformation:
        obj = new SLAM::Map();
        break;
    case BuddyMessage::SemanticInformation:
        obj = new SemanticMapInfoMessage();
        break;
    case BuddyMessage::HighCommand:
        obj = new HighLevelCommand();
        break;
    case BuddyMessage::InformationTransfer:
        obj = new InfoMessage();
        break;
    case BuddyMessage::AskBidsOnFrontiers:
        obj = new AskBidsMessage<SLAM::Geometry::Frontier>();
        break;
    case BuddyMessage::SendBidsOnFrontiers:
        obj = new SendBidsMessage();
        break;
    case BuddyMessage::FrontierAssignment:
        obj = new AssignItemMessage<SLAM::Geometry::Frontier>();
        break;
    case BuddyMessage::AskBidsOnVictims:
        obj = new AskBidsMessage<uint>();
        break;
    case BuddyMessage::SendBidsOnVictims:
        obj = new SendBidsMessage();
        break;
    case BuddyMessage::VictimAssignment:
        obj = new AssignItemMessage<uint>();
        break;
    case BuddyMessage::AcknowledgmentMessage:
        obj = new AckMessage();
        break;
    case BuddyMessage::WaypointMessage:
        obj = new WaypointCommand();
        break;
    case BuddyMessage::ModuleActivation:
        obj = new ModuleActivationMessage();
        break;
    case BuddyMessage::VictimConfirmation:
        obj = new VictimConfirmationMessage();
        break;
    case BuddyMessage::VictimDeletion:
        obj = new VictimDelectionMessage();
        break;
    case BuddyMessage::RobotVictimCoupling:
        obj = new RobotVictimCouplingMessage();
        break;
    case BuddyMessage::DistanceVector:
        obj = new DistanceVectorMessage();
        break;
    case BuddyMessage::DestinationNotificationMessage:
        obj = new DestinationMessage();
        break;
    case BuddyMessage::VictimDetectionConfiguration:
        obj = new VictimDetectionConfMessage();
        break;
    case BuddyMessage::ErrorNotification:
        obj = new ErrorNotificationMessage();
        break;
    default:
        raw = NULL;
        hasOwnership = false;
        return;
    }

    stream >> obj;
    hasOwnership = true;
    raw = obj;
}

LoggerStream &operator<<(LoggerStream &stream, BuddyMessage::Content c)
{
    switch(c){
    case BuddyMessage::Empty: stream << "BuddyMessage::Empty"; break;
    case BuddyMessage::WheelMotion: stream << "BuddyMessage::WheelMotion"; break;
    case BuddyMessage::AirMotion: stream << "BuddyMessage::AirMotion"; break;
    case BuddyMessage::EnableCameraStreaming: stream << "BuddyMessage::EnableCameraStreaming"; break;
    case BuddyMessage::DisableCameraStreaming: stream << "BuddyMessage::DisableCameraStreaming"; break;
    case BuddyMessage::CameraFrame: stream << "BuddyMessage::CameraFrame"; break;
    case BuddyMessage::RawUSARCommand: stream << "BuddyMessage::RawUSARCommand"; break;
    case BuddyMessage::VictimInformation: stream << "BuddyMessage::VictimInformation"; break;
    case BuddyMessage::MapInformation: stream << "BuddyMessage::MapInformation"; break;
    case BuddyMessage::SemanticInformation: stream << "BuddyMessage::SemanticInformation"; break;
    case BuddyMessage::HighCommand: stream << "BuddyMessage::HighCommand"; break;
    case BuddyMessage::InformationTransfer: stream << "BuddyMessage::InformationTransfer"; break;
    case BuddyMessage::AskBidsOnFrontiers: stream << "BuddyMessage::AskBidsOnFrontiers"; break;
    case BuddyMessage::SendBidsOnFrontiers: stream << "BuddyMessage::SendBidsOnFrontier"; break;
    case BuddyMessage::FrontierAssignment: stream << "BuddyMessage::FrontierAssignment"; break;
    case BuddyMessage::AskBidsOnVictims: stream << "BuddyMessage::AskBidsOnVictims"; break;
    case BuddyMessage::SendBidsOnVictims: stream << "BuddyMessage::SendBidsOnVictims"; break;
    case BuddyMessage::VictimAssignment: stream << "BuddyMessage::VictimAssignment"; break;
    case BuddyMessage::AcknowledgmentMessage: stream << "BuddyMessage::AckMessage"; break;
    case BuddyMessage::WaypointMessage: stream << "BuddyMessage::WaypointMessage"; break;
    case BuddyMessage::ModuleActivation: stream << "BuddyMessage::ModuleActivation"; break;
    case BuddyMessage::DistanceVector: stream << "BuddyMessage::DistanceVector"; break;
    case BuddyMessage::DestinationNotificationMessage: stream << "BuddyMessage::DestinationMessage"; break;
    case BuddyMessage::VictimDetectionConfiguration: stream << "BuddyMessage::VictimDetectionConfiguration"; break;
    case BuddyMessage::ErrorNotification: stream << "BuddyMessage::ErrorNotificationMessage"; break;
    default: stream << "Message of unknown content, or better, message with a content not yet added to the switch that prints typenames (buddymessage.cpp)";
    }
    return stream;
}
}
