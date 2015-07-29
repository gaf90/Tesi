#ifndef VICTIMMESSAGE_H
#define VICTIMMESSAGE_H

#include "serializable.h"
#include "message.h"
#include "pose.h"
#include <QString>
#include <QPoint>
#include <QImage>

namespace Data{
/**
 * @brief Victim detection message to Base Station
 *
 * VictimMessage is used as a communication container from robots to the base station.
 * It contains information about the robot generating the message (and that is supposed
 * to have find the victim), about the estimated position of the victim and about the
 * confidence of the detection process.
 *
 * @see
 */
class VictimMessage : public Message, public Serializable
{

public:

    VictimMessage();


    /**
     * Initializes a new VictimMessage with the required information
     *
     * @param robotName name of the robot sending the VictimMessage
     * @param confidence confidence of the victim detection process, it is a double between 0 and 1.
     * @param position estimation of the QPoint in wich the victim is.
     */
    VictimMessage(QString robotName, QImage image, double confidence, Pose position);

    /**
     * Destroys the VictimMessage
     */
    virtual ~VictimMessage();

    /**
     * @return The left confidence of the victim detection process
     */
    double getConfidence() const;

    /**
     * @return The estimated position of the victim
     */
    Pose getPosition() const;

    /**
     * @return The name of the robot that sends the message
     */
    QString getRobotName() const;

    /**
     * @return The name of the robot that sends the message
     */
    const QImage &getVictimImage() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    double confidence;
    Pose position;
    QString robotName;
    const QImage &img;
    QImage loadedFrame;
};
}
#endif // VICTIMMESSAGE_H
