#ifndef TESTFRONTIERAUCTION_H
#define TESTFRONTIERAUCTION_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test{
    class TestFrontierAuction : public QObject
    {
        Q_OBJECT
    public:
        explicit TestFrontierAuction(QObject *parent = 0);
        virtual ~TestFrontierAuction();

    signals:

    public slots:

    private slots:
        void testMessages();
        void testAssignFrontiers();
    };
}
#endif // TESTFRONTIERAUCTION_H
