#ifndef ROBOTVICTIMCOUPLINGMESSAGE_H
#define ROBOTVICTIMCOUPLINGMESSAGE_H

#include "qglobal.h"
#include "data/message.h"
#include "data/serializable.h"

namespace Data{
    class RobotVictimCouplingMessage: public Message, public Serializable
    {
    public:
        RobotVictimCouplingMessage();
        RobotVictimCouplingMessage(uint victimId, int timeToReach);
        ~RobotVictimCouplingMessage();
        uint getVictimId() const;
        int getTimeToReach() const;

        void serializeTo(QDataStream &stream) const;
        void deserializeFrom(QDataStream &stream);

    private:
        uint victimId; //qhash of the pose of the victim
        int timeToReach; //time to reach the victim
    };
}

#endif // ROBOTVICTIMCOUPLINGMESSAGE_H
