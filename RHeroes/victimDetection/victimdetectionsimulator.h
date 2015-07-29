#ifndef VICTIMDETECTIONSIMULATOR_H
#define VICTIMDETECTIONSIMULATOR_H

#include <QList>
#include <data/pose.h>

namespace VictimDetection {
class VictimDetectionSimulator
{
public:
    VictimDetectionSimulator();
    bool isThereAVictim(const Data::Pose &pose);

private:
    void loadVictimsPose();
    QList<Data::Pose> victimsPoseList;
};
}
#endif // VICTIMDETECTIONSIMULATOR_H
