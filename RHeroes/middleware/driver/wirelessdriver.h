#ifndef WIRELESSDRIVER_H
#define WIRELESSDRIVER_H

#include "sensordriver.h"
#include "data/wirelessmessage.h"
#include <QHash>
#include <QByteArray>

namespace Middleware {

class WirelessDriver : public SensorDriver
{
    Q_OBJECT

public:
    WirelessDriver(QObject *parent = 0, uint nrobot = 0);
    virtual ~WirelessDriver();

signals:
    void sigDistanceVector(const QString &sender, const QHash<QString, int> &dv);

public slots:
    /**
     * Driver::onDriverMessage() implementation for WirelessDriver
     */
    void onDriverMessage(const Data::Message &message);

    /**
     * SensorDriver::onMessageReceived() implementation for WirelessDriver
     */
    virtual void onMessageReceived(const Data::Message &message);

    void flushBuffer(const QString& robotName);

private:
    void sendMessage(const Data::WirelessMessage &msg,
                     const QString &destination);
    void sendMessageBroadcastRobot(const Data::WirelessMessage &msg);

    QHash<QString, QByteArray> partialData;
    uint nRobot;
};

}

#endif // WIRELESSDRIVER_H
