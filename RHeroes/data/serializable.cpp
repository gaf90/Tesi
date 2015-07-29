#include "serializable.h"
#include <QBuffer>
#include <stdint.h>

namespace Data {

Serializable::Serializable()
{
}

Serializable::~Serializable()
{
}

QDataStream &operator<<(QDataStream &stream, const Serializable *obj)
{
    return stream << *obj;
}

QDataStream &operator<<(QDataStream &stream, const Serializable &obj)
{
    QByteArray binData;
    QDataStream data(&binData, QIODevice::WriteOnly);
    obj.serializeTo(data);
    uint32_t size = binData.length();
    stream.setByteOrder(QDataStream::BigEndian);
    stream << size;
    stream.writeRawData(binData, binData.length());
    return stream;
}

QDataStream &operator>>(QDataStream &stream, Serializable *obj)
{
    return stream >> *obj;
}

QDataStream &operator>>(QDataStream &stream, Serializable &obj)
{
    uint32_t size_ignore;
    stream.setByteOrder(QDataStream::BigEndian);
    stream >> size_ignore;
    obj.deserializeFrom(stream);
    return stream;
}

}
