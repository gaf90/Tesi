#ifndef TESTRRTALGORITHM_H
#define TESTRRTALGORITHM_H

#include <QObject>
#include <QtTest/QtTest>
namespace Test{
class TestRRTAlgorithm : public QObject
{
    Q_OBJECT
public:
    explicit TestRRTAlgorithm(QObject *parent = 0);
    virtual ~TestRRTAlgorithm();
signals:
    
public slots:

private slots:
    void testComputeLowLevelAction();
    void testRandomNode();
    void testNearNode();
    void testClosestNode();
    void testConstrainedClosestNode();
    void testUnconstrainedComputeOrientation();
    void testConstrainedComputeOrientation();
    
};
}

#endif // TESTRRTALGORITHM_H
