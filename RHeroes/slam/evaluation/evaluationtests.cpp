/*
 * evaluationtests.cpp
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#include "evaluationtests.h"

namespace SLAM {
namespace Evaluation {

EvaluationTests::EvaluationTests(const QString &fname) : map(fname)
{
    lprint << "Graphics[" << map.map() << "]" << endl;
}

void EvaluationTests::runTests()
{
    distanceEvaluation();
    scanmatchingEvaluation();
    covarianceEvaluation();
}

} /* namespace Evaluation */
} /* namespace SLAM */
