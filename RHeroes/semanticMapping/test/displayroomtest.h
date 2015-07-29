#ifndef DISPLAYROOMTEST_H
#define DISPLAYROOMTEST_H

#include <QObject>
#include "slam/map.h"

namespace SemanticMapping{
    namespace Test{

class DisplayRoomTest : public QObject
{
    Q_OBJECT
public:
    explicit DisplayRoomTest(QObject *parent = 0);

    virtual ~DisplayRoomTest();
signals:
    
public slots:
    void luposFirstTest(SLAM::Map map);

private:
    int bbbbb;
};
    }
}
#endif // DISPLAYROOMTEST_H
