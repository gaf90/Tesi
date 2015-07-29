#ifndef UPISMESSAGE_H
#define UPISMESSAGE_H

#include "message.h"
#include <QByteArray>

namespace Data {

/**
 * @brief Message representing data sent from UPIS
 *
 * The UPISMessage class represents a message returned by UPIS. It is simply
 * a raw image byte array with the additional property of being able to
 * distinguish between JPEG and RAW image data.
 *
 * @see UPISController
 */
class UPISMessage : public Message, public QByteArray
{
public:
    /**
     * Initialises a new UPISMessage with data content binData
     *
     * @param jpeg true if the content is JPEG data, false if RAW data
     * @param binData Binary image data
     */
    UPISMessage(bool jpeg, const QByteArray &binData);
    virtual ~UPISMessage();

    /**
     * @return true if the binary data is in JPEG format, false otherwise
     */
    bool isJPEG() const;

    /**
     * Logical negation of isJPEG()
     *
     * @return true is the binary data is in RAW format, false otherwise
     */
    bool isRAW() const;

private:
    bool jpeg;
};

}

#endif // UPISMESSAGE_H
