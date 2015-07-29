#include "criterion.h"
#include <limits>

namespace Exploration{

using namespace SLAM;
using namespace SLAM::Geometry;

Criterion::Criterion()
{
}

Criterion::Criterion(const QString &name, double weight, bool highGood) :
    name(name), weight(weight),
    maxValue(std::numeric_limits<double>::min()), minValue(std::numeric_limits<double>::max()),
    highGood(highGood)
{
}

Criterion::~Criterion()
{
}

const QString & Criterion::getName() const
{
    return name;
}
double Criterion::getWeight() const
{
    return weight;
}

void Criterion::setName(const QString &name)
{
    this->name = name;
}

void Criterion::setWeight(double weight)
{
    this->weight = weight;
}

double Criterion::getEvaluation(Frontier *point) const
{
    return evaluation[point];
}

void Criterion::clean()
{
//    for(QHash<SLAM::Geometry::Frontier *, double>::iterator it = evaluation.begin(); it!=evaluation.end(); it++){
//        delete it.key();
//    }
    evaluation.clear();
}

void Criterion::insertEvaluation(const Frontier *point, double value)
{
    evaluation.insert(point, value);
    if(value >= maxValue)
        maxValue = value;
    if(value <= minValue)
        minValue = value;
}

void Criterion::normalizeHighGood()
{
    QHash<const Frontier *, double> temp;
    foreach(const Frontier *key, evaluation.keys()){
        double value = evaluation[key];
        value = (value-minValue)/(maxValue-minValue);
        temp.insert(key, value);
    }
    evaluation = temp;
}

void Criterion::normalizeLowGood()
{
    QHash<const Frontier *, double> temp;
    foreach(const Frontier *p, evaluation.keys()){
        double value = evaluation[p];
        value = (maxValue-value)/(maxValue-minValue);
        temp.insert(p, value);
    }
    evaluation = temp;
}

void Criterion::normalize()
{
    if(highGood)
        normalizeHighGood();
    else
        normalizeLowGood();
}

void Criterion::setWorstValue(Frontier *point)
{
    if(highGood){
        insertEvaluation(point, 0.0);
    } else {
        insertEvaluation(point, 1.0);
    }
}

}
