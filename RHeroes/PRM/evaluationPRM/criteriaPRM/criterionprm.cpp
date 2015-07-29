#include "criterionprm.h"
#include <limits>
#include "shared/config.h"

namespace PRM{

using namespace SLAM;
using namespace SLAM::Geometry;

CriterionPRM::CriterionPRM()
{
}

CriterionPRM::CriterionPRM(const QString &name, double weight, bool highGood) :
    name(name), weight(weight),
    maxValue(std::numeric_limits<double>::min()), minValue(std::numeric_limits<double>::max()),
    highGood(highGood)
{
}

CriterionPRM::~CriterionPRM()
{
}

const QString & CriterionPRM::getName() const
{
    return name;
}
double CriterionPRM::getWeight() const
{
    return weight;
}

void CriterionPRM::setName(const QString &name)
{
    this->name = name;
}

void CriterionPRM::setWeight(double weight)
{
    this->weight = weight;
}

QList<double> CriterionPRM::getEvaluation(Frontier *point) const
{
    return evaluation[point];
}

void CriterionPRM::clean()
{
    //    for(QHash<SLAM::Geometry::Frontier *, double>::iterator it = evaluation.begin(); it!=evaluation.end(); it++){
    //        delete it.key();
    //    }
    evaluation.clear();
}

void CriterionPRM::insertEvaluation(const Frontier* point, QList<double> value)
{
    evaluation.insert(point, value);
    foreach(double v, value){
        if(v >= maxValue)
            maxValue = v;
        if(v <= minValue)
            minValue = v;
    }
}

void CriterionPRM::normalizeHighGood()
{
    QHash<const Frontier* , QList<double> > temp;
    foreach(const Frontier* key, evaluation.keys()){
        QList<double> tempValue= QList<double>();
        QList<double> value = evaluation[key];
        foreach(double v, value){
            if((maxValue-minValue)==0){
                tempValue.append(1);
            }
            else{
                v = (v-minValue)/(maxValue-minValue);
                tempValue.append(v);
            }
        }
        temp.insert(key, tempValue);
    }
    evaluation = temp;
}

void CriterionPRM::normalizeLowGood()
{
    QHash<const Frontier* , QList<double> > temp;
    foreach(const Frontier* p, evaluation.keys()){
        QList<double> tempValue= QList<double>();
        QList<double> value = evaluation[p];
        foreach(double v, value){
            if(v==INFINITY){
                tempValue.append(0);
            }
            else if((maxValue-minValue)==0){
                tempValue.append(1);
            }
            else{
                v = (maxValue-v)/(maxValue-minValue);
                tempValue.append(v);
            }
        }
        temp.insert(p, tempValue);
    }
    evaluation = temp;
}

void CriterionPRM::normalize()
{
    if(highGood)
        normalizeHighGood();
    else
        normalizeLowGood();
}

void CriterionPRM::setWorstValue(const Frontier *point)
{
    QList<double> worstValue;
    if(highGood){
        for(int i=0; i<Config::PRM::pathNumber;i++){
            worstValue.append(0.0);
        }
        insertEvaluation(point, worstValue);
    } else {
        for(int i=0; i<Config::PRM::pathNumber;i++){
            worstValue.append(INFINITY);
        }
        //insertEvaluation(point, worstValue);
        evaluation.insert(point,worstValue);
    }
}

bool CriterionPRM::isHighGood(){
    return highGood;
}

}
