#ifndef TESTEVALUATIONFUNCTIONS_H
#define TESTEVALUATIONFUNCTIONS_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test {
    class TestEvaluationFunctions : public QObject
    {
        Q_OBJECT
    public:
        explicit TestEvaluationFunctions(QObject *parent = 0);
        virtual ~TestEvaluationFunctions();

    signals:

    public slots:

    private slots:
        void testComputeInformationGain();
        void testComputeDistanceFromOtherRobots();
        void testComputeTransmissionProbability();

    };
}

#endif // TESTEVALUATIONFUNCTIONS_H
