#ifndef TESTEXPLORATIONMODULE_H
#define TESTEXPLORATIONMODULE_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test {
class TestExplorationModule : public QObject
{
public:
    explicit TestExplorationModule(QObject *parent = 0);
    virtual ~TestExplorationModule();

private slots:
    void TestSearchNewFrontiers();
};
}

#endif // TESTEXPLORATIONMODULE_H
