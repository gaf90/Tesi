/*
 * ransacmatcher.h
 *
 *  Created on: 06/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef RANSACMATCHER_H_
#define RANSACMATCHER_H_

#include "shared/config.h"
#include "slam/utilities.h"
#include "scanmatcher.h"
#include "decoupledestimator.h"
#include "minlengthweightingstrategy.h"
#include "overlap.h"
#include "shared/random.h"
#include "slam/dataset/debugconfiguration.h"
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
#   include "slam/dataset/mathematicavisualization.h"
#endif

namespace SLAM {
namespace ScanMatching {

template <typename S, typename A, typename W = MinLengthWeightingStrategy>
class RANSACMatcher : public ScanMatcher<S> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RANSACMatcher();
    virtual ~RANSACMatcher();
	bool run();
	Eigen::Matrix3d covariance();

private:
	struct Association {
	    int scanidx, assocrefidx;
	};

    bool buildDistribution();
    double associationDensityWeight(int scanidx, int refidx) const;

    void fillRototranslations();
    bool significant(const Eigen::Vector2d &deviation) const;
    double associationImportanceWeight(
            int scanidx, int refidx, const Geometry::Rototranslation &rt);

private:
	DecoupledEstimator<S, W> estimator;
	Shared::StepwiseCDF<Association> cdf;
	Eigen::Matrix<double, 3, Eigen::Dynamic> *rts;
};


class AssociationAmigoni;
class AssociationElseberg;
class AssociationLiGriffiths;
class AssociationProbabilistic;
class AssociationProbabilisticOld;

namespace __internal {

#define DETECTOR_ENUM_BODY(a, e, lg, p)     \
    enum {                                  \
        is_amigoni = a,                     \
        is_elseberg = e,                    \
        is_ligriffiths = lg,                \
        is_probabilistic = p                \
    };

/* Association detector class */
template <typename T> struct detector                       { DETECTOR_ENUM_BODY(0, 0, 0, 0) };
template <> struct detector<AssociationAmigoni>             { DETECTOR_ENUM_BODY(1, 0, 0, 0) };
template <> struct detector<AssociationElseberg>            { DETECTOR_ENUM_BODY(0, 1, 0, 0) };
template <> struct detector<AssociationLiGriffiths>         { DETECTOR_ENUM_BODY(0, 0, 1, 0) };
template <> struct detector<AssociationProbabilistic>       { DETECTOR_ENUM_BODY(0, 0, 0, 1) };

#undef DETECTOR_ENUM_BODY

} /* namespace __internal */

template <typename S, typename A, typename W>
RANSACMatcher<S, A, W>::RANSACMatcher() : rts(NULL)
{

}

template <typename S, typename A, typename W>
RANSACMatcher<S, A, W>::~RANSACMatcher()
{
    delete[] rts;
}

template <typename S, typename A, typename W>
Eigen::Matrix3d RANSACMatcher<S, A, W>::covariance()
{
    return estimator.covariance();
}

template <typename S, typename A, typename W>
bool RANSACMatcher<S, A, W>::run()
{
    const int querySegmentCount = this->retriever->querySegmentCount();

    QList<int> *bestAssoc = new QList<int>[querySegmentCount];
    QList<int> *testAssoc = new QList<int>[querySegmentCount];

    double bestImportanceSum = 0;
    this->rt = this->retriever->queryPose();

    if(!buildDistribution()) {
        delete[] bestAssoc;
        delete[] testAssoc;
        return false;
    } else {
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
        ldbg << "(* Association pool *)" << endl;
        Dataset::MathematicaVisualization::associationMap(*this->retriever, this->assoc);
#endif
        estimator.setRetriever(*this->retriever);

        fillRototranslations();

#if 0
        int distinctCount = 0;
        for(int i = 0; i < querySegmentCount; i++) {
           if(this->assoc[i].size() > 0)
               distinctCount++;
        }
        const bool singleAssociation = (distinctCount == 1);
#endif
/*
        int assocCount = 0;
        for(int i = 0; i < querySegmentCount; i++) {
           assocCount += this->assoc[i].size();
        }
        lprint << assocCount << endl;
*/
#if 1
        for(int k = 0, h = 0; k < Config::SLAM::ransacMinimumIterations &&
                h < Config::SLAM::ransacMaximumIterations; k++, h++) {
            Association a1, a2;
            a1 = Shared::Random::sample(cdf);
            //do {
            a2 = Shared::Random::sample(cdf);
            //} while(a1.scanidx == a2.scanidx && !singleAssociation);
#else
        for(int k = 0, h = 0; h < cdf.stepCount() * cdf.stepCount(); h++) {
            const Association &a1 = cdf[h % cdf.stepCount()];
            const Association &a2 = cdf[h / cdf.stepCount()];
#endif
            int a1refidx = this->assoc[a1.scanidx][a1.assocrefidx];
            int a2refidx = this->assoc[a2.scanidx][a2.assocrefidx];

            testAssoc[a1.scanidx].append(a1refidx);
            testAssoc[a2.scanidx].append(a2refidx);

            estimator.setAssociations(testAssoc);
            estimator.estimate();

#if 0
            //Dataset::MathematicaVisualization::associationMap(*this->retriever, testAssoc);
            ldbg << "Graphics[{Red," << this->retriever->querySegment(a1.scanidx) << "," <<
                    this->retriever->querySegment(a2.scanidx) << ",Black," <<
                    this->retriever->mapSegment(a1refidx) << "," <<
                    this->retriever->mapSegment(a2refidx) << "}]" << endl;

            Dataset::MathematicaVisualization::translationMap(estimator, *this->retriever, testAssoc);
            ldbg << "(* " << a1.scanidx << "," << a1.assocrefidx << " " << a2.scanidx << "," << a2.assocrefidx << " *)" << endl;
            ldbg << endl;
#endif

            if(significant(estimator.singleAssociationDeviation(a1.scanidx, a1refidx)) ||
                    significant(estimator.singleAssociationDeviation(a2.scanidx, a2refidx))) {
                testAssoc[a1.scanidx].clear();
                testAssoc[a2.scanidx].clear();
                k--;
                continue;
            }

            const Eigen::Vector3d guessRT = estimator.pureRototranslation();

            for(int i = 0; i < querySegmentCount; i++) {
                for(int j = 0; j < this->assoc[i].size(); j++) {
                    if(!significant(estimator.deviation(rts[i].block<3, 1>(0, j), guessRT)) &&
                            (i != a1.scanidx || j != a1.assocrefidx) &&
                            (i != a2.scanidx || j != a2.assocrefidx)) {
                        testAssoc[i].append(this->assoc[i][j]);
                    }
                }
            }

            estimator.estimate();
            Geometry::Rototranslation guessRTobj = estimator.rototranslation();

            //const double importanceSum = overlapAmount(*this->retriever, guessRTobj, this->assoc);

            double importanceSum = 0;
            for(int i = 0; i < querySegmentCount; i++) {
                for(int j = 0; j < testAssoc[i].size(); j++) {
                    importanceSum += associationImportanceWeight(i, testAssoc[i][j], guessRTobj);
                }
            }

#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
            //Dataset::MathematicaVisualization::associationMap(*this->retriever, testAssoc);
            //Dataset::MathematicaVisualization::translationMap(estimator, *this->retriever, testAssoc);

            //ldbg << "(* " << a1.scanidx << "," << a1.assocrefidx << " " << a2.scanidx << "," << a2.assocrefidx << " *)" << endl;
            //ldbg << "Print[" << importanceSum << "];" << endl;
#endif

            if(importanceSum > bestImportanceSum) {
                bestImportanceSum = importanceSum;
                std::swap(bestAssoc, testAssoc);
            }

            for(int i = 0; i < querySegmentCount; i++) {
                testAssoc[i].clear();
            }
        }

        estimator.setAssociations(bestAssoc);
        estimator.estimate();
        this->rt = Geometry::Rototranslation(estimator.rototranslation()) *
                this->retriever->queryPose();
        this->retriever->applyTransformation(estimator.rototranslation());
        delete[] testAssoc;
        delete[] this->assoc;
        this->assoc = bestAssoc;
#if defined(PLOT_ASSOCIATIONS) && PLOT_ASSOCIATIONS
        ldbg << "(* Final associations *)" << endl;
        Dataset::MathematicaVisualization::associationMap(*this->retriever, this->assoc);
        Dataset::MathematicaVisualization::translationMap(estimator, *this->retriever, this->assoc);
#endif
        return true;
    }
}

template <typename S, typename A, typename W>
bool RANSACMatcher<S, A, W>::buildDistribution()
{
   int count = A::broadLookup(*this->retriever, this->assoc);
   if(count == 0) {
       return false;
   } else {
       int querySegmentCount = this->retriever->querySegmentCount();
       QList<double> densityWeights;
       QList<Association> associations;

       for(int i = 0; i < querySegmentCount; i++) {
            for(int j = 0; j < this->assoc[i].size(); j++) {
                Association a = { i, j };
                densityWeights.append(
                        this->associationDensityWeight(i, this->assoc[i][j]));
                associations.append(a);
            }
       }
       cdf = Shared::StepwiseCDF<Association>(densityWeights, associations);
       return true;
   }
}

template <typename S, typename A, typename W>
bool RANSACMatcher<S, A, W>::significant(const Eigen::Vector2d &deviation) const
{
    //return deviation[0] > M_PI / 10 || deviation[1] > 0.15;
    return deviation[0] > M_PI / 20 || deviation[1] > 0.08;
}

template <typename S, typename A, typename W>
inline double RANSACMatcher<S, A, W>::associationImportanceWeight(
        int scanidx, int refidx, const Geometry::Rototranslation &rt)
{
    return overlapAmount(*this->retriever, rt, scanidx, refidx);
}

template <typename S, typename A, typename W>
inline double RANSACMatcher<S, A, W>::associationDensityWeight(int scanidx, int refidx) const
{
#if 0
    return std::min(
            this->retriever->derived().querySegment(scanidx).length(),
            this->retriever->derived().mapSegment(refidx).length());
#endif
    /* These weights don't really have a theory behind */
    double d = A::distance(this->retriever->derived(), scanidx, refidx);
    if(__internal::detector<A>::is_probabilistic) {
        /*
             Probabilistic-tuned variant:
             If it's in the 3 sigma range return the defined maximum (5), otherwise decrease
             linearly up to a minimum of 1
        */
        return max(min(9 - 4 * std::sqrt(d) / 3, 5.), 1.);
    } else if(__internal::detector<A>::is_amigoni || __internal::detector<A>::is_elseberg) {
        /* Very rough Amigoni/Elseberg variant */
        if(d < 0.2) {
            return 5;
        } else if(d < 0.3) {
            return 3;
        } else if(d < 0.5) {
            return 2;
        } else {
            return 1;
        }
    } else {
        /* Otherwise equal weights */
        return 1;
    }
}

template <typename S, typename A, typename W>
void RANSACMatcher<S, A, W>::fillRototranslations()
{
    int querySegmentCount = this->retriever->querySegmentCount();
    delete[] rts;
    rts = new Eigen::Matrix<double, 3, Eigen::Dynamic>[querySegmentCount];

    for(int i = 0; i < querySegmentCount; i++) {
        rts[i] = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, this->assoc[i].size());

        for(int j = 0; j < this->assoc[i].size(); j++) {
            rts[i].block<3, 1>(0, j) = estimator.singleAssociationEstimate(i, this->assoc[i][j]);
        }
    }
}

} /* namespace ScanMatching */
} /* namespace SLAM */
#endif /* RANSACMATCHER_H_ */
