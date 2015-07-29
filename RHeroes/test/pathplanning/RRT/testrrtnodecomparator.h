#ifndef TESTRRTNODECOMPARATOR_H
#define TESTRRTNODECOMPARATOR_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test {
    class TestRRTnodeComparator : public QObject
    {
        Q_OBJECT
    public:
        explicit TestRRTnodeComparator(QObject *parent = 0);
        virtual ~TestRRTnodeComparator();
    signals:
    private slots:
        void testCompare();

    public slots:

    };
}

#endif // TESTRRTNODECOMPARATOR_H
