#ifndef HYBRIDPOSEACTION_H
#define HYBRIDPOSEACTION_H

#include "data/pose.h"
#include "pathPlanner/abstractaction.h"

namespace PathPlanner{
class HybridPoseAction : public AbstractAction
{
public:
    HybridPoseAction(const Data::Pose &pose);
    virtual ~HybridPoseAction();

    const Data::Pose & getValue() const;

private:
    Data::Pose p;

};

}

#endif // HYBRIDPOSEACTION_H
