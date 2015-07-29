/*
 * classicicl.h
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef CLASSICICL_H_
#define CLASSICICL_H_

#include "slam/constants.h"
#include "slam/utilities.h"
#include "scanmatcher.h"
#include "decoupledestimator.h"
#include "minlengthweightingstrategy.h"

namespace SLAM {
namespace ScanMatching {

template <typename S, typename A, typename W = MinLengthWeightingStrategy>
class ClassicICL : public ScanMatcher<S> {
public:
	bool run();
    Eigen::Matrix3d covariance();

private:
    DecoupledEstimator<S, W> estimator;
};

template <typename S, typename A, typename W>
Eigen::Matrix3d ClassicICL<S, A, W>::covariance()
{
    return estimator.covariance();
}

template <typename S, typename A, typename W>
bool ClassicICL<S, A, W>::run()
{
    int counter = 0;
    Eigen::Vector3d stepRT;

    estimator.setRetriever(*this->retriever);

    do {
        bool foundSomething = A::template lookup<1>(*this->retriever, this->assoc);
        if(!foundSomething) {
            this->rt = this->retriever->queryPose();
            return false;
        }
        estimator.setAssociations(this->assoc);
        estimator.estimate();
        stepRT = estimator.rototranslation();
        this->retriever->applyTransformation(Geometry::Rototranslation(stepRT));
        counter++;
    } while((SQUARE(stepRT[0]) + SQUARE(stepRT[1]) > SQUARE(SM_ICL_CONVERGENCE_THRESHOLD) ||
            std::abs(wrap(stepRT[2])) > SM_ICL_CONVERGENCE_THRESHOLD) &&
            counter < SM_ICL_MAXIMUM_ITERATIONS);

    this->rt = this->retriever->queryPose();
	return true;
}

} /* namespace ScanMatching */
} /* namespace SLAM */
#endif /* CLASSICICL_H_ */
