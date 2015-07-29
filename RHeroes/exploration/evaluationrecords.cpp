#include "evaluationrecords.h"

using namespace SLAM;
using namespace Geometry;

namespace Exploration {
    EvaluationRecords::EvaluationRecords(uint robotId) :
        evaluations(new QHash<Frontier, double>()), robotId(robotId)
    {

    }

    EvaluationRecords::EvaluationRecords() :
        evaluations(new QHash<Frontier, double>())
    {

    }

    EvaluationRecords::~EvaluationRecords()
    {
        delete evaluations;
    }

    void EvaluationRecords::putEvaluation(Frontier frontier, double value)
    {
        if(evaluations->isEmpty()){
            minValue = value;
            maxValue = value;
        }

        evaluations->insert(frontier, value);

        if(value >= maxValue)
            maxValue = value;
        if(value <= minValue)
            minValue = value;


    }

    double EvaluationRecords::getEvaluation(Frontier frontier)
    {
        return evaluations->value(frontier);
    }

    bool EvaluationRecords::contains(Frontier frontier)
    {
        return evaluations->contains(frontier);
    }

    int EvaluationRecords::size()
    {
        return evaluations->size();
    }

    uint EvaluationRecords::getRobotId()
    {
        return robotId;
    }

    void EvaluationRecords::setRobotId(uint robotId){
        this->robotId = robotId;
    }

    QList<SLAM::Geometry::Frontier> *EvaluationRecords::getEvaluatedFrontiers() {
        QList<SLAM::Geometry::Frontier> list = evaluations->keys();
        QList<SLAM::Geometry::Frontier> *toRet = new QList<SLAM::Geometry::Frontier>();

        foreach(Frontier f, list){
            toRet->append(f);
        }

        return toRet;
    }

    void EvaluationRecords::serializeTo(QDataStream &stream) const
    {
        stream << robotId;
        stream << *evaluations;
    }

    void EvaluationRecords::deserializeFrom(QDataStream &stream)
    {
        stream >> robotId;
        stream >> *evaluations;
    }

    QHash<SLAM::Geometry::Frontier, double> * EvaluationRecords::getEvaluations()
    {
        return evaluations;
    }


    QHash<uint, double> EvaluationRecords::getEvaluationFrontiersBids()
    {
        QHash<uint, double> evaluationsRecords;
        foreach(SLAM::Geometry::Frontier tempFrontier, evaluations->keys()){
            evaluationsRecords.insert(qHash(tempFrontier), (*evaluations)[tempFrontier]);
        }
        return evaluationsRecords;
    }


    void EvaluationRecords::removeFrontier(Frontier frontier)
    {        
        foreach(SLAM::Geometry::Frontier tempFrontier, evaluations->keys()){
            if(tempFrontier == frontier){
                evaluations->remove(tempFrontier);
            }
        }
    }

    void EvaluationRecords::normalize(){
       QList<SLAM::Geometry::Frontier> list = evaluations->keys();
       foreach(Frontier f, list){
           double value = evaluations->value(f);
           value = (value-minValue)/(maxValue-minValue);
           evaluations->remove(f);
           evaluations->insert(f, value);
       }
    }

}
