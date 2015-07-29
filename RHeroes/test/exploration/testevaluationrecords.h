#ifndef TESTEVALUATIONRECORDS_H
#define TESTEVALUATIONRECORDS_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test {
class TestEvaluationRecords : public QObject
{
    Q_OBJECT
public:
    explicit TestEvaluationRecords(QObject *parent = 0);
    virtual ~TestEvaluationRecords();
signals:
    
public slots:

private slots:
    void TestPutEvaluation();
    void TestGetEvaluation();
    void TestGetEvaluations();
    void TestContains();
    void TestSize();
    void TestGetFrontiers();
    void TestRemoveFrontier();
    
};
}

#endif // TESTEVALUATIONRECORDS_H
