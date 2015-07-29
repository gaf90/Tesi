#ifndef RRTNODECOMPARATOR_H
#define RRTNODECOMPARATOR_H

#include "rrtnode.h"

namespace PathPlanner{
    class RRTNodeComparator
    {
    public:
        RRTNodeComparator(RRTNode * reference);
        virtual ~RRTNodeComparator();

        bool operator()(const RRTNode *n1, const RRTNode *n2);
    private:
        RRTNode * ref;
    };
}

#endif // RRTNODECOMPARATOR_H
