#include "hybridposeaction.h"

namespace PathPlanner{

using namespace Data;

HybridPoseAction::HybridPoseAction(const Pose &pose)
    : p(pose)
{
}
HybridPoseAction::~HybridPoseAction()
{
}

const Pose & HybridPoseAction::getValue() const
{
    return p;
}


}
