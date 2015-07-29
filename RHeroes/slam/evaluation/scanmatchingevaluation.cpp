/*
 * distanceevaluation.cpp
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#include "evaluationtests.h"
#include "slam/scanmatching/associationposecentric.h"
#include "slam/scanmatching/associationelseberg.h"
#include "slam/engine/semideterministicretriever.h"
#include "slam/support/stopwatch.h"
#include "slam/scanmatching/ligriffithsicl.h"
#include "slam/scanmatching/classicicl.h"
#include "slam/scanmatching/filteredicl.h"
#include "slam/scanmatching/ransacmatcher.h"
#include "slam/scanmatching/uniformweightingstrategy.h"
#include "slam/scanmatching/minlengthweightingstrategy.h"
#include "slam/scanmatching/mlweightingstrategy.h"
#include "shared/random.h"
#include "concurrentrunner.h"
#include "truthmap.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigen>
#include <QMutex>

namespace SLAM {
namespace Evaluation {

using namespace ScanMatching;
using namespace Geometry;
using namespace Engine;
using namespace Data;
using namespace Support;

typedef AssociationElseberg AssociationType;

enum WeightingSelection {
    Uniform,
    MinLength,
    MaximumLikelihood
};

struct ScanMatchingConfiguration {
    TruthMap *map;
    Eigen::Matrix3d covariance;
    WeightingSelection weighting;
    int sampleCount;
};

struct MeasureError {
    MeasureError(double x, double y, double t) : x(x), y(y), t(t) {}
    double x, y, t;
};

static LoggerStream &operator<<(LoggerStream &stream, const ScanMatchingConfiguration &conf) {
    stream << "Weighting evaluation profile: ";
    switch(conf.weighting) {
    case Uniform: stream << "uniform"; break;
    case MinLength: stream << "minimum length"; break;
    case MaximumLikelihood: stream << "maximum likelihood"; break;
    }
    return stream;
}

static LoggerStream &operator<<(LoggerStream &stream, const MeasureError &err) {
    return stream << "{" << err.x << "," << err.y << "," << err.t << "}";
}


template <typename S>
void doScanMatch(
        SemiDeterministicRetriever &sdr, const Pose &realPose, const Eigen::Matrix3d &cov,
        int &correctCount, QList<MeasureError> &errors) {
    Q_UNUSED(cov)
    Eigen::Vector3d beforeErr = realPose.vectorForm() - sdr.initialQueryPose().vectorForm();
    S matcher;
    matcher.setRetriever(sdr);
    matcher.run();
    Eigen::Vector3d afterErr = realPose.vectorForm() - matcher.measure();
    beforeErr[2] = wrap(beforeErr[2]);
    afterErr[2] = wrap(afterErr[2]);
    /*if((double)(beforeErr.transpose() * cov.inverse() * beforeErr) >
        (double)(afterErr.transpose() * cov.inverse() * afterErr)) {
        correctCount++;
        errors.append(MeasureError(afterErr[0], afterErr[1], afterErr[2]));
    }*/

    double xydist = afterErr.block<2,1>(0,0).norm(), tdist = std::abs(afterErr[2]);
    if(xydist < 0.2 && tdist < 5 * M_PI / 180 && matcher.associationCount() > 0) {
        correctCount++;
    }
    errors.append(MeasureError(afterErr[0], afterErr[1], afterErr[2]));
}

template <typename W>
void evaluateWithWeighting(ScanMatchingConfiguration &conf) {
    const TruthMap &map = *conf.map;
    QList<MeasureError> errors[4];
    int correctCounts[4] = {0, 0, 0, 0};
    static QMutex mutex;

    Eigen::Matrix3d sqrtCov = conf.covariance.llt().matrixL();
    Eigen::Matrix3d newCov  = 4 * conf.covariance;

    QList<LineSegment *> segments = map.map();

    for(int i = 0; i < conf.sampleCount; i++) {
        int pickedIndex = Shared::Random::integer(0, map.nodeCount() - 1);
        const SegmentScan *scan = map.scan(pickedIndex);
        const Pose &pose = map.pose(pickedIndex);
        const UncertainRototranslation corruptedPose(
                Rototranslation(Shared::Random::multiNormalRoot(
                        pose.vectorForm(), sqrtCov)), newCov);

        SemiDeterministicRetriever sdr(segments, *scan, corruptedPose);

        doScanMatch<LiGriffithsICL<SemiDeterministicRetriever> >(sdr, pose, newCov, correctCounts[0], errors[0]);
        doScanMatch<ClassicICL<SemiDeterministicRetriever, AssociationType, W> >(sdr, pose, newCov, correctCounts[1], errors[1]);
        doScanMatch<FilteredICL<SemiDeterministicRetriever, AssociationType, W> >(sdr, pose, newCov, correctCounts[2], errors[2]);
        doScanMatch<RANSACMatcher<SemiDeterministicRetriever, AssociationType, W> >(sdr, pose, newCov, correctCounts[3], errors[3]);

    }

    mutex.lock();
    lprint << conf << endl;

    lprint << "ligriffithsAccuracy=" << correctCounts[0] / (double) conf.sampleCount << ";" << endl;
    lprint << "classiciclAccuracy=" << correctCounts[1] / (double) conf.sampleCount << ";" << endl;
    lprint << "filterediclAccuracy=" << correctCounts[2] / (double) conf.sampleCount << ";" << endl;
    lprint << "ransacmatcherAccuracy=" << correctCounts[3] / (double) conf.sampleCount << ";" << endl;
    lprint << "ligriffithsErrors=" << errors[0] << ";" << endl;
    lprint << "classiciclErrors=" << errors[1] << ";" << endl;
    lprint << "filterediclErrors=" << errors[2] << ";" << endl;
    lprint << "ransacmatcherErrors=" << errors[3] << ";" << endl;
    lprint << endl;
    mutex.unlock();
}

void singleConfigurationEvaluation(ScanMatchingConfiguration &conf) {
    switch(conf.weighting) {
    case Uniform:
        evaluateWithWeighting<UniformWeightingStrategy>(conf);
        break;
    case MinLength:
        evaluateWithWeighting<MinLengthWeightingStrategy>(conf);
        break;
    case MaximumLikelihood:
        evaluateWithWeighting<MLWeightingStrategy>(conf);
        break;
    }
}

template <typename S>
void doScanMatchTiming(
        SemiDeterministicRetriever &sdr, double &assocTotalTime, double &scanTotalTime) {
    static QMutex perche;
    perche.lock();
    S matcher;
    sdr.stopwatch.reset();
    Stopwatch scanTotal;
    scanTotal.start();
    matcher.setRetriever(sdr);
    matcher.run();
    scanTotal.stop();
    assocTotalTime += sdr.stopwatch.time();
    scanTotalTime += scanTotal.time();
    perche.unlock();
}

template <typename W>
void timingWithWeighting(ScanMatchingConfiguration &conf) {
    const TruthMap &map = *conf.map;
    double assocTimings[4] = {0, 0, 0, 0}, scanTimings[4] = {0, 0, 0, 0};
    static QMutex mutex;

    Eigen::Matrix3d sqrtCov = conf.covariance.llt().matrixL();
    Eigen::Matrix3d newCov  = conf.covariance;

    QList<LineSegment *> segments = map.map();

    for(int i = 0; i < conf.sampleCount; i++) {
        int pickedIndex = Shared::Random::integer(0, map.nodeCount() - 1);
        const SegmentScan *scan = map.scan(pickedIndex);
        const Pose &pose = map.pose(pickedIndex);
        const UncertainRototranslation corruptedPose(
                Rototranslation(Shared::Random::multiNormalRoot(
                        pose.vectorForm(), sqrtCov)), newCov);

        SemiDeterministicRetriever sdr(segments, *scan, corruptedPose);

        doScanMatchTiming<LiGriffithsICL<SemiDeterministicRetriever> >(sdr, assocTimings[0], scanTimings[0]);
        doScanMatchTiming<ClassicICL<SemiDeterministicRetriever, AssociationType, W> >(sdr, assocTimings[1], scanTimings[1]);
        doScanMatchTiming<FilteredICL<SemiDeterministicRetriever, AssociationType, W> >(sdr, assocTimings[2], scanTimings[2]);
        doScanMatchTiming<RANSACMatcher<SemiDeterministicRetriever, AssociationType, W> >(sdr, assocTimings[3], scanTimings[3]);
    }

    mutex.lock();
    lprint << conf << endl;
    lprint << "ligriffithsAssociationTiming=" << assocTimings[0] / conf.sampleCount << ";" << endl;
    lprint << "classiciclAssociationTiming=" << assocTimings[1] / conf.sampleCount << ";" << endl;
    lprint << "filterediclAssociationTiming=" << assocTimings[2] / conf.sampleCount << ";" << endl;
    lprint << "ransacmatcherAssociationTiming=" << assocTimings[3] / conf.sampleCount << ";" << endl;
    lprint << "ligriffithsScanTiming=" << scanTimings[0] / conf.sampleCount << ";" << endl;
    lprint << "classiciclScanTiming=" << scanTimings[1] / conf.sampleCount << ";" << endl;
    lprint << "filterediclScanTiming=" << scanTimings[2] / conf.sampleCount << ";" << endl;
    lprint << "ransacmatcherScanTiming=" << scanTimings[3] / conf.sampleCount << ";" << endl;
    lprint << endl;
    mutex.unlock();
}

void singleConfigurationTiming(ScanMatchingConfiguration &conf) {
    switch(conf.weighting) {
    case Uniform:
        timingWithWeighting<UniformWeightingStrategy>(conf);
        break;
    case MinLength:
        timingWithWeighting<MinLengthWeightingStrategy>(conf);
        break;
    case MaximumLikelihood:
        timingWithWeighting<MLWeightingStrategy>(conf);
        break;
    }
}

void EvaluationTests::scanmatchingEvaluation() {
    int iters = 100;
    Eigen::Matrix3d largeCov = Eigen::Vector3d(.1, .1, square(M_PI / 12)).asDiagonal();

    QList<ScanMatchingConfiguration> args;

    ScanMatchingConfiguration uniform = { &map, largeCov, Uniform, iters };
    ScanMatchingConfiguration minlen  = { &map, largeCov, MinLength, iters };
    ScanMatchingConfiguration ml      = { &map, largeCov, MaximumLikelihood, iters };
    args << uniform << minlen << ml;

    //lprint << map.map() << endl << endl;

    //singleConfigurationEvaluation(minlen);
    ConcurrentRunner<ScanMatchingConfiguration, singleConfigurationEvaluation> runner1(3, args);
    ConcurrentRunner<ScanMatchingConfiguration, singleConfigurationTiming> runner2(3, args);
}

} /* namespace Evaluation */
} /* namespace SLAM */


