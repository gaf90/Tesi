#include "victimdetectionsimulator.h"
//#define MAP_NAME VMAC
#define VICTIM_DETECTED_MAX_DISTANCE 5

namespace VictimDetection{
    VictimDetectionSimulator::VictimDetectionSimulator()
    {
        loadVictimsPose();
    }

    bool VictimDetectionSimulator::isThereAVictim(const Data::Pose &pose)
    {
        foreach(Data::Pose tempPose, victimsPoseList){
            if(pose.getDistance(tempPose) < VICTIM_DETECTED_MAX_DISTANCE)
                return true;
        }
        return false;
    }

    void VictimDetectionSimulator::loadVictimsPose()
    {
#if (MAP_NAME == VMAC)
//        victimsPoseList.append(Data::Pose (-6.000000,2.950000,3));
//        victimsPoseList.append(Data::Pose (-30.900000,9.100000,1));
//        victimsPoseList.append(Data::Pose (-21.950000,16.600000,1));
//        victimsPoseList.append(Data::Pose (3.350000,21.100000,1));
#endif
    }
}
