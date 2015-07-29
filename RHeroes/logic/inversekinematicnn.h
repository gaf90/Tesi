#ifndef INVERSEKINEMATICNN_H
#define INVERSEKINEMATICNN_H

#include "data/pose.h"
#include "wheelspeeds.h"
#include "inversekinematic.h"
#include "doublefann.h"
#include "fann_cpp.h"

class InverseKinematicNN : public InverseKinematic
{
public:
    InverseKinematicNN();
    virtual ~InverseKinematicNN();

    const WheelSpeeds computeSpeeds(const Data::Pose &destPose) const;

private:

    struct fann* nn;
};

#endif // INVERSEKINEMATICNN_H
