#include "usarmessage.h"
#include <QRegExp>

namespace Data {

USARMessage::USARMessage()
{
}

USARMessage::USARMessage(const QString &line)
{
    QString qline = line.trimmed();
    int spaceIdx = qline.indexOf(' ');

    QRegExp rx("\\{([^ ]+) ([^\\}]+)\\}");
    int pos = spaceIdx;

    while ((pos = rx.indexIn(qline, pos)) != -1)
    {
        insert(rx.cap(1), rx.cap(2));
        pos += rx.matchedLength();
    }

    type = qline.left(spaceIdx);
}

USARMessage::USARMessage(const USARMessage &message) :
    Message(), Serializable(),
    type(message.type), data(message.data), order(message.order)
{
}


USARMessage::~USARMessage()
{
}

void USARMessage::setType(const QString &type)
{
    this->type = type;
}

const QString &USARMessage::getType() const
{
    return this->type;
}

USARMessage::operator QString() const
{
    QString serialised = type;
    foreach(QString key, order) {
        serialised += QString(" {%1 %2}").arg(key).arg(data[key]);
    }
    serialised += "\r\n";
    return serialised;
}

QString &USARMessage::operator[](const QString &key) {
    if(data.contains(key)) {
        return data[key];
    } else {
        insert(key, QString());
        return data[key];
    }
}

void USARMessage::insert(const QString &key, const QString &value) {
    order.append(key);
    data.insert(key, value);
}

bool USARMessage::contains(const QString &key) const {
    return data.contains(key);
}

const QList<QString> &USARMessage::keys() const {
    return order;
}

void USARMessage::serializeTo(QDataStream &stream) const
{
    stream << type << order << data;
}

void USARMessage::deserializeFrom(QDataStream &stream)
{
    stream >> type >> order >> data;
}

}
