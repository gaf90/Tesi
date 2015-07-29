#ifndef TESTSUITE_H
#define TESTSUITE_H

#include <QObject>
#include <QtTest/QtTest>

namespace Test{
class TestSuite : public QObject
{
    Q_OBJECT
public:
    explicit TestSuite(QObject *parent = 0);
    
signals:
    
public slots:
    
};
}

#endif // TESTSUITE_H
