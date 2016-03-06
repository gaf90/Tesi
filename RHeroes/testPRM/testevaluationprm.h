#ifndef TESTEVALUATIONPRM_H
#define TESTEVALUATIONPRM_H

#include <QObject>
#include <QtTest/QTest>

namespace TestPRM{


class TestEvaluationPRM : public QObject
{
    Q_OBJECT
public:
    explicit TestEvaluationPRM(QObject *parent = 0);
    
signals:
    
public slots:
    
private slots:
    void testEval();
    void testEval2();
};

}
#endif // TESTEVALUATIONPRM_H
