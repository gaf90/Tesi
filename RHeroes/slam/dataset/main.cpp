/*
 * main.cpp
 *
 *  Created on: 28/nov/2012
 *      Author: Mladen Mazuran
 */

#include "debugconfiguration.h"
#include "shared/configreader.h"

#if defined(EVALUATION_TESTING)
#   include "slam/evaluation/truthmap.h"
#   include "slam/evaluation/evaluationtests.h"
#elif defined(SCANMATCHING_TEST) && SCANMATCHING_TEST
#   include "exportedtest.h"
#   include "slam/geometry/segmentscan.h"
#   include "slam/geometry/quadrilateral.h"
#   include "slam/geometry/scanbox.h"
#   include "slam/scanmatching/ligriffithsicl.h"
#   include "slam/scanmatching/filteredicl.h"
#   include "slam/scanmatching/ransacmatcher.h"
#   include "slam/scanmatching/decoupledestimator.h"
#   include "slam/scanmatching/associationelseberg.h"
#   include "slam/scanmatching/associationprobabilistic.h"
#   include "slam/scanmatching/associationposecentric.h"
#   include "slam/scanmatching/minlengthweightingstrategy.h"
#   include "slam/engine/semideterministicretriever.h"
#   include "slam/engine/odometrycovariancemodels.h"
#else
#   include "datasetplayer.h"
#   include <QApplication>
#endif
#include <shared/config.h>

#if !defined(EVALUATION_TESTING) && SCANMATCHING_TEST
using namespace SLAM;
using namespace ScanMatching;
using namespace Geometry;
using namespace Engine;

//typedef FilteredICL<SemiDeterministicRetriever, AssociationProbabilistic> Matcher;
typedef RANSACMatcher<SemiDeterministicRetriever, AssociationPoseCentric> Matcher;
//typedef LiGriffithsICL<SemiDeterministicRetriever> Matcher;
typedef BasicOdometryCovarianceModel OdometryCovarianceModel;

QString disk(const Point &p) {
    return QString("Disk[{%1,%2},0.1]").arg(p.x(), 0, 'f', 5).arg(p.y(), 0, 'f', 5);
}
#endif

int main(int argc, char *argv[])
{
    Config::ConfigReader reader("poaret.conf");
    reader.readFileAndCompileConfigs();

    logger.setOutputFormat(Logger::PrependNothing);

#if defined(EVALUATION_TESTING)
#	ifndef DATASET_PREFIX
#		define DATASET_PREFIX ""
#	endif

    Q_UNUSED(argc) Q_UNUSED(argv)

    SLAM::Evaluation::EvaluationTests tests1(DATASET_PREFIX "csail.corrected.clf");
    SLAM::Evaluation::EvaluationTests tests2(DATASET_PREFIX "bicocca-rawseeds-gtruth-extended.clf");

    tests1.runTests();
    lprint << endl << "-----------------------------------------------------------------" << endl;
    tests2.runTests();

    return 0;
#elif !defined(SCANMATCHING_TEST) || !SCANMATCHING_TEST
    QApplication a(argc, argv);
    SLAM::Dataset::DatasetPlayer w;
    w.showMaximized();
    return a.exec();
#else
    Q_UNUSED(argc) Q_UNUSED(argv)

#if 1
    Map map = getDatasetMap();
    PointScan ps = PointScan::medianFilter(getDatasetScan(), 5);
    SegmentScan ss(ps);

    Eigen::Matrix3d cov;
    cov <<
            0.010000,0.000000,0.000000,
            0.000000,0.010000,0.000000,
            0.000000,0.000000,0.068539;

    UncertainRototranslation guess = Rototranslation(*map.lastRobotPose(0)) *
                    OdometryCovarianceModel::addCovariance(getDatasetDeltaOdometry());
    SemiDeterministicRetriever retriever(map.walls(), ss, guess);

    Matcher m;
    m.setRetriever(retriever);
    m.run();

    ldbg << "Graphics[{Red," << (Rototranslation(m.measure()) * ss).getSegments() <<
            ",Black," << map.walls() << "}]" << endl;
    ldbg << "Show[ListPlot[" << ps << "],Graphics[" << ss.getSegments() << "]]" << endl;
    ldbg << (Rototranslation(*map.lastRobotPose(0)) * Rototranslation(getDatasetDeltaOdometry())).vectorForm() << endl;
#endif

    return 0;
#endif
}
