#ifndef USARMESSAGE_H
#define USARMESSAGE_H

#include <QList>
#include <QHash>
#include <QDataStream>
#include "message.h"
#include "serializable.h"

namespace Data {

/**
 * @brief Message representing data sent to and from USARSim
 *
 * The USARMessage class represents a message aimed towards, or signalled by
 * USARController. The class is effectively a hash table containing key-value
 * pairs and an additional string representing the type of the message
 * (e.g. INFO, SEN, STA, DRIVE, ...).
 * When parsing a message the order of the parameters is preserved; similarly
 * when constructing a new message with the provided methods the ordering of
 * the parameters is determined by the order in which the key/value pairs were
 * inserted
 *
 * @see USARController
 */
class USARMessage : public Message, public Serializable
{
private:
    QString type;
    QHash<QString, QString> data;
    QList<QString> order;

public:

    /**
     * Initialises an empty USARMessage
     */
    USARMessage();

    /**
     * Initialises an USARMessage from a string serialised representation of
     * a USARSim message, the input is parsed and the type and key-value pairs
     * are filled
     *
     * @param line The input string serialised USARSim message
     */
    explicit USARMessage(const QString &line);

    /**
     * Initialises an USARMessage from another existing one
     *
     * @param message The USARMessage to clone
     */
    USARMessage(const USARMessage &message);

    /**
     * Destroys the USARMessage
     */
    virtual ~USARMessage();

    /**
     * Serialises the USARMessage into a valid string which adheres to the
     * USARSim standard
     *
     * @return The serialised string
     */
    operator QString() const;

    /**
     * Sets the message type
     *
     * @param type Message type
     */
    void setType(const QString &type);

    /**
     * Retrieves the message type
     *
     * @return Message type
     */
    const QString &getType() const;

    /**
     * Returns a constant reference to the value of the parameter specified by
     * key
     *
     * @param key Input parameter key
     * @return Constant reference to value indexed by key
     */
    const QString operator[](const QString &key) const;

    /**
     * Returns a modifiable reference to the value of the parameter specified by
     * key. If the key does not exist a new entry is created
     *
     * @param key Input parameter key
     * @return Modifiable reference to value indexed by key
     */
    QString &operator[](const QString &key);

    /**
     * Adds a new key/value pair to the message after the existing ones
     * (ordered)
     *
     * @param key Input parameter key
     * @param value Value associated to the key
     */
    void insert(const QString &key, const QString &value);

    /**
     * Checks whether a parameter key exists in the message or not
     *
     * @param key Input parameter key
     * @return true if key exists in the message, false otherwise
     */
    bool contains(const QString &key) const;

    /**
     * Provides const reference to the list of parameter keys in the message,
     * sorted in order of insertion (or parsing)
     *
     * @return Ordered parameter keys
     */
    const QList<QString> &keys() const;


    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

};

const inline QString USARMessage::operator[](const QString &key) const
{
    return data[key];
}

}

#endif // USARMESSAGE_H
