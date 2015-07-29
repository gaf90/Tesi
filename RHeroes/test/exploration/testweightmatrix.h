#ifndef TESTWEIGHTMATRIX_H
#define TESTWEIGHTMATRIX_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test{
    class TestWeightMatrix : public QObject
    {
        Q_OBJECT
    public:
        TestWeightMatrix();
        virtual ~TestWeightMatrix();

    signals:

    private slots:
        void testCreation();
        void testInsertAndGetSingle();
        void testComputeEncoding();
        void testInsertCompoundWeight();
        void testInsertCompoundWeightFromList();
        void testGetActiveCriteria();
        void testChangeCriterionState();
    };

}

#endif // TESTWEIGHTMATRIX_H
