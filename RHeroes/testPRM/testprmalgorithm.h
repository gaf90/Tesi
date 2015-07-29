#ifndef TESTPRM_H
#define TESTPRM_H

#include <QObject>
#include <QtTest/QtTest>
#include "shared/config.h"

namespace TestPRM{

class TestPRMAlgorithm: public QObject
{
    Q_OBJECT
public:
    explicit TestPRMAlgorithm(QObject *parent = 0);
    virtual ~TestPRMAlgorithm();
signals:

public slots:
private slots:
    void testRandomPoint();
    void testVisibilityCheckNewMap();
    void testVisibilityCheckFrontierNotPassed();
    void testVisibilityCheckFrontierPassed();
    void testVisibilityCheckFrontier();
    void testFoundFrontiers();
    void testPRMUpdate();
};
}
#endif // TESTPRM_H
