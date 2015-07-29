#ifndef TESTVICTIMAUCTION_H
#define TESTVICTIMAUCTION_H

#include <QObject>
#include <QtTest>
#include "coordination/auction/victimauction.h"

namespace Test{
class TestVictimAuction : public QObject
{
    Q_OBJECT
public:
    explicit TestVictimAuction(QObject *parent = 0);

signals:

private slots:
    void testVictimAllocation();

private:
    void addBids(Coordination::VictimAuction &victimAuction, QList<QList<double> > &bidsList, int robotCount, int victimCount);
};
}
#endif // TESTVICTIMAUCTION_H
