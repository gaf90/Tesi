/*
 * evaluationtests.h
 *
 *  Created on: 24/mar/2013
 *      Author: Mladen Mazuran
 */

#ifndef EVALUATIONTESTS_H_
#define EVALUATIONTESTS_H_

#include <QObject>
#include "truthmap.h"

namespace SLAM {
namespace Evaluation {

class EvaluationTests: public QObject {
    Q_OBJECT
public:
    EvaluationTests(const QString &fname);
    void distanceEvaluation();
    void covarianceEvaluation();
    void scanmatchingEvaluation();

public slots:
    void runTests();

private:
    TruthMap map;
};

} /* namespace Evaluation */
} /* namespace SLAM */

#endif /* EVALUATIONTESTS_H_ */
