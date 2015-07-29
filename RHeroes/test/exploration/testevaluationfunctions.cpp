#include "testevaluationfunctions.h"
#include "slam/geometry/frontier.h"
#include "exploration/aojrffunction.h"

namespace Test {

using namespace SLAM;
using namespace Geometry;
using namespace Exploration;

TestEvaluationFunctions::TestEvaluationFunctions(QObject *parent) :
    QObject(parent)
{
}

TestEvaluationFunctions::~TestEvaluationFunctions()
{

}

void TestEvaluationFunctions::testComputeInformationGain()
{
    Point p1(-0.5, -0.5), p2(1.0, 0.0);
    Frontier f(p1, p2);
    Map map;
    AOJRFFunction evalFun(0);

    double distance = evalFun.computeInformationGain(f, map);
    QVERIFY2(distance == p1.distance(p2), "The length of the frontier should be equal to the distance of the points");
}

void TestEvaluationFunctions::testComputeDistanceFromOtherRobots()
{

}

void TestEvaluationFunctions::testComputeTransmissionProbability()
{

//    Point p1(-0.5, -0.5), p2(1.0, 0.0);
//    Frontier f(p1, p2);
//    Map map;
//    AOJRFFunction evalFun(0);
//    QHash<uint, double> signalPowerData();

//    double prob = evalFun.computeTransmissionProbability(f, map, signalPowerData());

}

}
