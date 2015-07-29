#include "evaluationrecordsprm.h"

namespace PRM{

using namespace Exploration;

EvaluationRecordsPRM::EvaluationRecordsPRM(): EvaluationRecords(),evalPRM(new QHash<Frontier, PRMPath>())
{
}


EvaluationRecordsPRM::EvaluationRecordsPRM(uint robotID): EvaluationRecords(robotID),evalPRM(new QHash<Frontier, PRMPath>())
{

}

PRMPath EvaluationRecordsPRM::getPath(Frontier frontier){
    return evalPRM->value(frontier);
}

void EvaluationRecordsPRM::putEvaluation(Frontier frontier, double value, PRMPath path){
    evalPRM->insert(frontier, path);
    EvaluationRecords::putEvaluation(frontier,value);
}

}
