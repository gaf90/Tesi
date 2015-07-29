#ifndef TESTWEIGHTREADER_H
#define TESTWEIGHTREADER_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test {
    class TestWeightReader : public QObject
    {
        Q_OBJECT
    public:
        TestWeightReader();
        virtual ~TestWeightReader();

    signals:
    public slots:
    private slots:
        void testCreateMatrix();

    };
}
#endif // TESTWEIGHTREADER_H
