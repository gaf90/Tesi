#ifndef TESTAUCTION_H
#define TESTAUCTION_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test{
    class TestAuction : public QObject
    {
        Q_OBJECT
    public:
        explicit TestAuction(QObject *parent = 0);
        virtual ~TestAuction();

    signals:

    public slots:

    private slots:
        void testAddItems();
        void testAddBids();

    };
}

#endif // TESTAUCTION_H
