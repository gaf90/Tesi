#ifndef EVALUATIONRECORDSPRM_H
#define EVALUATIONRECORDSPRM_H
#include "exploration/evaluationrecords.h"
#include "PRM/aStarPRM/astaralgorithmprm.h"

namespace PRM{

using namespace Exploration;

class EvaluationRecordsPRM : EvaluationRecords
{

public:
    EvaluationRecordsPRM(uint robotID);
    EvaluationRecordsPRM();
    PRMPath getPath(Frontier frontier);
    void putEvaluation(Frontier frontier, double value, PRMPath path);

private:
    QHash<Frontier,PRMPath> *evalPRM;
};

}
#endif // EVALUATIONRECORDSPRM_H
