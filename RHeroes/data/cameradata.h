#ifndef CAMERADATA_H
#define CAMERADATA_H

#include "message.h"
#include "serializable.h"
#include <QImage>

namespace Data {

/**
 * @brief Camera data message
 *
 * This class contains a camera frame, signalled by CameraSensor.
 *
 * @see CameraSensor
 */
class CameraData : public Message, public Serializable
{
public:

    CameraData();

    /**
     * Constructs a new CameraData message, containing a camera frame
     *
     * @param frame The camera frame
     */
    CameraData(const QImage &frame);

    /**
     * Destroys the CameraData
     */
    virtual ~CameraData();

    /**
     * @return A const reference to the camera frame
     */
    const QImage &getFrame() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    QImage loadedFrame;
    const QImage &frame;
};

}

#endif // CAMERADATA_H
