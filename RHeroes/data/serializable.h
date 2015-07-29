#ifndef SERIALIZABLE_H
#define SERIALIZABLE_H

#include <QDataStream>

namespace Data {

class Serializable
{
public:
    Serializable();
    virtual ~Serializable();

    virtual void serializeTo(QDataStream &stream) const = 0;
    virtual void deserializeFrom(QDataStream &stream) = 0;

    friend QDataStream &operator<<(
        QDataStream &stream, const Serializable *obj);
    friend QDataStream &operator<<(
        QDataStream &stream, const Serializable &obj);
    friend QDataStream &operator>>(
        QDataStream &stream, Serializable *obj);
    friend QDataStream &operator>>(
        QDataStream &stream, Serializable &obj);
};

}

#endif // SERIALIZABLE_H
