/*
 * filteredicl.h
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef FILTEREDICL_H_
#define FILTEREDICL_H_

#include "slam/constants.h"
#include "slam/utilities.h"
#include "scanmatcher.h"
#include "decoupledestimator.h"
#include "retriever.h"
#include "minlengthweightingstrategy.h"
#include "slam/dataset/debugconfiguration.h"
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
#   include "slam/dataset/mathematicavisualization.h"
#endif

namespace SLAM {
namespace ScanMatching {

template <typename S, typename A, typename W = MinLengthWeightingStrategy>
class FilteredICL : public ScanMatcher<S> {
public:
	bool run();
    Eigen::Matrix3d covariance();

private:
	struct DeviationEntry {
	    double devRotation, devTranslation;
	    int  scanidx, refidx;

	    bool operator<(const DeviationEntry &d) const {
	        return devTranslation < d.devTranslation;
	    }
	};

	inline bool significant(const Eigen::Vector2d &deviation) {
        return deviation[0] > M_PI / 20 || deviation[1] > 0.08;
	    //return deviation[0] > M_PI / 10 || deviation[1] > 0.15;
	}

private:
    DecoupledEstimator<S, W> estimator;
};

template <typename S, typename A, typename W>
Eigen::Matrix3d FilteredICL<S, A, W>::covariance()
{
    return estimator.covariance();
}

template <typename S, typename A, typename W>
bool FilteredICL<S, A, W>::run()
{
    int counter = 0;
    int querySegments = this->retriever->querySegmentCount();
    Eigen::Vector3d stepRT;

    estimator.setRetriever(*this->retriever);

    do {
        //ldbg << ">>> Iter start" << endl;
        bool foundSomething = A::template lookup<1>(*this->retriever, this->assoc);
        if(!foundSomething) {
            this->rt = this->retriever->queryPose();
            return false;
        }

        estimator.setAssociations(this->assoc);
        estimator.estimate();
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
        ldbg << "(* Before filtering *)" << endl;
        Dataset::MathematicaVisualization::associationMap(*this->retriever, this->assoc);
        Dataset::MathematicaVisualization::translationMap(estimator, *this->retriever, this->assoc);
#endif

        QList<DeviationEntry> deviations;
        for(int i = 0; i < querySegments; i++) {
            for(QList<int>::const_iterator it = this->assoc[i].begin(),
                    end = this->assoc[i].end(); it != end; ++it) {
                Eigen::Vector2d dev = estimator.singleAssociationDeviation(i, *it);
                if(significant(dev)) {
                    DeviationEntry de = {dev[0], dev[1], i, *it};
                    deviations.append(de);
                }
            }
        }

        //ldbg << estimator.rototranslation() << endl;
        while(deviations.size() > 0) {
            qSort(deviations);
            DeviationEntry &de = deviations.last();
            estimator.removeAssociation(de.scanidx, de.refidx);
            //ldbg << estimator.getAt() << " " << estimator.getBt() << " -> " << estimator.rototranslation() << endl;
            //estimator.estimate();
            //ldbg << estimator.getAt() << " " << estimator.getBt() << " -> " << estimator.rototranslation() << endl;
            deviations.removeLast();

            for(int i = deviations.size() - 1; i >= 0; i--) {
                DeviationEntry &de = deviations[i];
                Eigen::Vector2d dev = estimator.singleAssociationDeviation(de.scanidx, de.refidx);

                if(significant(dev)) {
                    de.devRotation = dev[0];
                    de.devTranslation = dev[1];
                } else {
                    deviations.removeAt(i);
                }
            }
        }
        //estimator.estimate();
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
        ldbg << "(* After filtering *)" << endl;
        Dataset::MathematicaVisualization::associationMap(*this->retriever, this->assoc);
        Dataset::MathematicaVisualization::translationMap(estimator, *this->retriever, this->assoc);
#endif
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
#endif /* FILTEREDICL_H_ */
