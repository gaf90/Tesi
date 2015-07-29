#ifndef TESTASTARPRM_H
#define TESTASTARPRM_H

#include <QObject>
#include <QtTest/QtTest>

namespace TestPRM{

class TestAStarPRM : public QObject
{
    Q_OBJECT
public:
    explicit TestAStarPRM(QObject *parent = 0);
    
signals:
    
public slots:
    
private slots:
    void testPath();
    void testPath2();


};
}
#endif // TESTASTARPRM_H
