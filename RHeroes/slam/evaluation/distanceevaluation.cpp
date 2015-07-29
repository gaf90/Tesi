/*
 * distanceevaluation.cpp
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#include "evaluationtests.h"
#include "slam/scanmatching/associationligriffiths.h"
#include "slam/scanmatching/associationamigoni.h"
#include "slam/scanmatching/associationelseberg.h"
#include "slam/scanmatching/associationprobabilistic.h"
#include "slam/scanmatching/associationposecentric.h"
#include "slam/engine/semideterministicretriever.h"
#include "slam/support/stopwatch.h"
#include "shared/random.h"
#include "concurrentrunner.h"
#include "truthmap.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigen>
#include <QMutex>
#include <QList>

namespace SLAM {
namespace Evaluation {

using namespace ScanMatching;
using namespace Geometry;
using namespace Engine;
using namespace Data;

struct EvalConfiguration {
    QString profileName;
    TruthMap *map;

    double ligriffithsThreshold, amigoniThreshold, elsebergThreshold;
    double probabilisticThreshold, poseCentricThreshold;

    Eigen::Matrix3d covariance;

    int sampleCount;
};

static LoggerStream &operator<<(LoggerStream &stream, const EvalConfiguration &conf) {
    return stream <<
            "Distance evaluation profile: " << conf.profileName /*<< endl <<
            "    Li & Griffiths threshold: " << conf.ligriffithsThreshold << endl <<
            "    Amigoni threshold:        " << conf.amigoniThreshold << endl <<
            "    Elseberg threshold:       " << conf.elsebergThreshold << endl <<
            "    Probabilistic threshold:  " << conf.probabilisticThreshold << endl <<
            "    Pose centric threshold:   " << conf.poseCentricThreshold << endl <<
            "    Pose covariance:          " << conf.covariance << endl <<
            "    Sample evaluations:       " << conf.sampleCount << endl*/;
}

template <typename A>
int updateCounts(
        SemiDeterministicRetriever &sdr, const SegmentScan *scan, double threshold,
        long &bestCount, long &correctCount, long &associatedCount, double &associatedCount2) {
    QList<int> *assoc = new QList<int>[sdr.querySegmentCount()];
    int validLineSegments = 0;
    A::template lookup<30>(threshold, sdr, assoc);
    for(int i = 0; i < sdr.querySegmentCount(); i++) {
        if(sdr.querySegment(i).length() > 0.3)
            validLineSegments++;
        else
            continue;
        if(assoc[i].size() > 0) {
            const MarkedLineSegment &best = static_cast<const MarkedLineSegment &>(
                    sdr.mapSegment(assoc[i][0]));
            if(best.contains(scan, i)) {
                bestCount++;
                correctCount++;
                associatedCount += assoc[i].size();
                associatedCount2 += square(assoc[i].size());
            } else {
                for(int j = 1; j < assoc[i].size(); j++) {
                    const MarkedLineSegment &mls = static_cast<const MarkedLineSegment &>(
                            sdr.mapSegment(assoc[i][j]));
                    if(mls.contains(scan, i)) {
                        correctCount++;
                        associatedCount += assoc[i].size();
                        associatedCount2 += square(assoc[i].size());
                        break;
                    }
                }
            }
        }
    }
    delete[] assoc;
    return validLineSegments;
}

void singleConfigurationDistanceEvaluation(EvalConfiguration &conf) {
    const TruthMap &map = *conf.map;
    long bestCounts[5], correctCounts[5], associatedCounts[5];
    double associatedCounts2[5];
    long lineSegmentCount = 0;
    static QMutex mutex;

    for(int i = 0; i < 5; i++) {
        bestCounts[i] = 0;
        correctCounts[i] = 0;
        associatedCounts[i] = 0;
        associatedCounts2[i] = 0;
    }

    Eigen::Matrix3d sqrtCov = conf.covariance.llt().matrixL();
    //double maxSingularValue = std::sqrt(conf.covariance.eigenvalues().real().maxCoeff());
    Eigen::Matrix3d newCov  = conf.covariance;

    QList<LineSegment *> segments = map.map();

    for(int i = 0; i < conf.sampleCount; i++) {
        int pickedIndex = Shared::Random::integer(0, map.nodeCount() - 1);
        const SegmentScan *scan = map.scan(pickedIndex);
        const Pose &pose = map.pose(pickedIndex);
        const UncertainRototranslation corruptedPose(
                Rototranslation(Shared::Random::multiNormalRoot(
                        pose.vectorForm(), sqrtCov)), newCov);

#if 0
        double maxrange = 0;
        fforeach(const LineSegment &s, scan->getSegments()) {
            if(s.p1().norm() > maxrange) {
                maxrange = s.p1().norm();
            } else if(s.p2().norm()) {
                maxrange = s.p2().norm();
            }
        }

        QList<LineSegment *> segments = map.portion(
                Point(corruptedPose.translation()), maxrange + 10 * maxSingularValue);
#endif
        SemiDeterministicRetriever sdr(segments, *scan, corruptedPose);
        lineSegmentCount +=
        updateCounts<AssociationLiGriffiths  >(sdr, scan, conf.ligriffithsThreshold,
                bestCounts[0], correctCounts[0], associatedCounts[0], associatedCounts2[0]);
        /*updateCounts<AssociationAmigoni      >(sdr, scan, conf.amigoniThreshold,
                bestCounts[1], correctCounts[1], associatedCounts[1], associatedCounts2[1]);
        updateCounts<AssociationElseberg     >(sdr, scan, conf.elsebergThreshold,
                bestCounts[2], correctCounts[2], associatedCounts[2], associatedCounts2[2]);
        updateCounts<AssociationProbabilistic>(sdr, scan, conf.probabilisticThreshold,
                bestCounts[3], correctCounts[3], associatedCounts[3], associatedCounts2[3]);
        updateCounts<AssociationPoseCentric  >(sdr, scan, conf.poseCentricThreshold,
                bestCounts[4], correctCounts[4], associatedCounts[4], associatedCounts2[4]);*/
    }

    double bestFrequency[5], correctFrequency[5], associatedAvg[5], associatedStdDev[5];
    for(int i = 0; i < 5; i++) {
        associatedAvg[i]    = associatedCounts[i] / (double) correctCounts[i];
        associatedStdDev[i] = std::sqrt((associatedCounts2[i] - correctCounts[i] * associatedAvg[i]) /
                                        (correctCounts[i] - 1) / correctCounts[i]);
        bestFrequency[i]    = bestCounts[i] / (double) lineSegmentCount;
        correctFrequency[i] = correctCounts[i] / (double) lineSegmentCount;
    }

    mutex.lock();
    lprint << conf << endl;
    lprint << "    Li & Griffiths: " << bestFrequency[0] << "\t" << correctFrequency[0] << "\t" << associatedAvg[0] << " (" << associatedStdDev[0] << ")" << endl;
    lprint << "    Amigoni:        " << bestFrequency[1] << "\t" << correctFrequency[1] << "\t" << associatedAvg[1] << " (" << associatedStdDev[1] << ")" << endl;
    lprint << "    Elseberg:       " << bestFrequency[2] << "\t" << correctFrequency[2] << "\t" << associatedAvg[2] << " (" << associatedStdDev[2] << ")" << endl;
    lprint << "    Probabilistic:  " << bestFrequency[3] << "\t" << correctFrequency[3] << "\t" << associatedAvg[3] << " (" << associatedStdDev[3] << ")" << endl;
    lprint << "    Pose centric:   " << bestFrequency[4] << "\t" << correctFrequency[4] << "\t" << associatedAvg[4] << " (" << associatedStdDev[4] << ")" << endl;
    lprint << "    Sample count:   " << lineSegmentCount << endl;
    lprint << endl;
    mutex.unlock();
}

template <typename A>
double updateTiming(SemiDeterministicRetriever &sdr) {
    QList<int> *assoc = new QList<int>[sdr.querySegmentCount()];
    Support::Stopwatch stopwatch;
    stopwatch.start();
    A::template lookup<1>(sdr, assoc);
    stopwatch.stop();
    delete[] assoc;
    return stopwatch.time() / (sdr.querySegmentCount() * sdr.mapSegmentCount());
}


void timingTest(const TruthMap &map, int sampleCount, const Eigen::Matrix3d &covariance) {
    QList<double> timings[5];

    QList<LineSegment *> segments = map.map();

    for(int i = 0; i < sampleCount; i++) {
        int pickedIndex = Shared::Random::integer(0, map.nodeCount() - 1);
        const SegmentScan *scan = map.scan(pickedIndex);
        const Pose &pose = map.pose(pickedIndex);
        const UncertainRototranslation uncertainPose(pose, covariance);

        SemiDeterministicRetriever sdr(segments, *scan, uncertainPose);
        timings[0].append(updateTiming<AssociationLiGriffiths  >(sdr) * 1e6);
        /*timings[1].append(updateTiming<AssociationAmigoni      >(sdr) * 1e6);
        timings[2].append(updateTiming<AssociationElseberg     >(sdr) * 1e6);
        timings[3].append(updateTiming<AssociationProbabilistic>(sdr) * 1e6);
        timings[4].append(updateTiming<AssociationPoseCentric  >(sdr) * 1e6);*/
    }

    lprint << "timingsLiGriffiths=" << timings[0] << ";" << endl;
    lprint << "timingsAmigoni=" << timings[1] << ";" << endl;
    lprint << "timingsElseberg=" << timings[2] << ";" << endl;
    lprint << "timingsProbabilistic=" << timings[3] << ";" << endl;
    lprint << "timingsPoseCentric=" << timings[4] << ";" << endl;
    lprint << endl;
}



#define REPEAT5(a) a[0], a[1], a[2], a[3], a[4]
#define PUT_CONF(size1, size2) EvalConfiguration size1##_##size2 = {    \
    #size1 " covariance, " #size2 " thresholds", &map,                  \
    REPEAT5(size2##Thresholds), size1##Cov, iters                       \
}; args << size1##_##size2

void EvaluationTests::distanceEvaluation() {
    int iters = 10000;
    double smallThresholds[]    = { .05, .1, .1, 9, 9 };
    double mediumThresholds[]   = { .15, .2, .2, 16, 16 };
    double largeThresholds[]    = { .5, .5, .5, 25, 25 };
    Eigen::Matrix3d smallCov    = Eigen::Vector3d(4e-4, 4e-4, square(M_PI / 120)).asDiagonal();
    Eigen::Matrix3d mediumCov   = Eigen::Vector3d(4e-3, 4e-3, square(M_PI / 120) * 10).asDiagonal();
    Eigen::Matrix3d largeCov    = Eigen::Vector3d(4e-2, 4e-2, square(M_PI / 12)).asDiagonal();

    QList<EvalConfiguration> args;

    PUT_CONF(small,  small);
    PUT_CONF(small,  medium);
    PUT_CONF(small,  large);
    PUT_CONF(medium, small);
    PUT_CONF(medium, medium);
    PUT_CONF(medium, large);
    PUT_CONF(large,  small);
    PUT_CONF(large,  medium);
    PUT_CONF(large,  large);

    ConcurrentRunner<EvalConfiguration, singleConfigurationDistanceEvaluation> runner(4, args);
    timingTest(map, iters, largeCov);
}

} /* namespace Evaluation */
} /* namespace SLAM */


