#ifndef TESTRRTNODE_H
#define TESTRRTNODE_H

#include <QObject>
#include <QtTest/QtTest>


namespace Test{
class TestRRTNode : public QObject
{
    Q_OBJECT
public:
    explicit TestRRTNode(QObject *parent = 0);
    virtual ~TestRRTNode();

signals:
    
public slots:

    void testEquals();
    void testNotEquals();
    void testDistance();
private slots:
    
};
}
#endif // TESTRRTNODE_H
