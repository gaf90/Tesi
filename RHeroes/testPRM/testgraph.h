#ifndef TESTGRAPH_H
#define TESTGRAPH_H

#include <QObject>
#include <QtTest/QtTest>

namespace TestPRM{

class TestGraph : public QObject
{
    Q_OBJECT
public:
    explicit TestGraph(QObject *parent = 0);
    virtual ~TestGraph();
    
signals:
    
public slots:
private slots:
    void testGraphNode();
    void testGraphEdge();
    void testNewGraph();
};
}

#endif // TESTGRAPH_H
