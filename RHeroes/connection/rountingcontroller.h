#ifndef ROUNTINGCONTROLLER_H
#define ROUNTINGCONTROLLER_H

#include <QObject>
#include <QHash>
#include <QTimer>
#include "data/distancevectormessage.h"
#include "shared/constants.h"

namespace Connection {

class RoutingController : public QObject
{
    Q_OBJECT
public:
    explicit RoutingController(QObject *parent = 0, uint nRobot = 0);
    
    void updateDirectConnection(QString peer, double signalStrength);
    void processReceivedDistanceVector(const QString &peer, const QHash<QString, int> &dv);
    QString getNextHopDestination(const QString &dest);

signals:
    void signalRoutingUpdated(const QHash<QString, int> &newDistanceVector);
    void signalNewConnectedPeer(QString peer);
    void signalNewReachablePeer(QString peer);

private slots:
    void updateDistanceVector();
    //void updateDistanceVector(QString other);

    //void onUpdateHoldDown();

private:
    typedef struct{
        QString nextHop;
        int cost; //Measured as number of hops needed to reach the destination.
    } t_connection;

    uint nrobot;
    QHash<QString /*destination*/, t_connection> routingTable;

    //QTimer* clearHoldDown;

    //QHash<QString,QHash<QString, qint64 > > holdDown;
    /*typedef struct{
        QString dest;
        QString nextHop;
        qint64 until;
    }HoldDown;
    QList<HoldDown> holdDown;*/
    QTimer* distanceVectorTimer;

    //bool checkHoldDown(const QString& dest,const QString& nextHop);

};

}
#endif // ROUNTINGCONTROLLER_H
