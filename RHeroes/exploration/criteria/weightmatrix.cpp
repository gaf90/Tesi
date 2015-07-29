#include "weightmatrix.h"
#include "shared/logger.h"

namespace Exploration{
WeightMatrix::WeightMatrix(int numOfCriteria) :
    activeCriteria(new QHash<QString, bool>()), mapping(new QHash<QString, QString>()),
    weights(new QList<QHash<QString, double> *>()), lastInsertedCriteria(0), numOfActiveCriteria(0),
    mutex(new QMutex())
{
    for(int i=0; i<numOfCriteria-1; i++){
        weights->append(new QHash<QString, double>());
    }

}

WeightMatrix::~WeightMatrix()
{
    delete mutex;
    for(int i=weights->size()-1; i>=0; i--){
        delete weights->at(i);
    }
    delete weights;
    mapping->clear();
    delete mapping;
    activeCriteria->clear();
    delete activeCriteria;
}

void WeightMatrix::insertSingleCriterion(QString name, double weight, bool active)
{
    mutex->lock();
    lastInsertedCriteria++;
    QString code(QString::number(lastInsertedCriteria));
    mapping->insert(name, code);
    activeCriteria->insert(code, active);
    if(active)
        numOfActiveCriteria++;
    weights->at(0)->insert(code, weight);
    mutex->unlock();
}

void WeightMatrix::changeCriteriaActivation(const QString &name, bool active)
{
    mutex->lock();
    QString enc = mapping->value(name);
    bool actualState = activeCriteria->value(enc);
    if(actualState == active){
        mutex->unlock();
        return;
    }
    activeCriteria->insert(enc, active);
    if(active)
        numOfActiveCriteria++;
    else
        numOfActiveCriteria--;
    mutex->unlock();
}

double WeightMatrix::getWeight(QList<QString> criteriaNames) const
{

    QString enc = computeNamesEncoding(criteriaNames);
    double w = getWeight(enc);
    if (w == 0){
        //no encoding saved. I must compute the weight by summing
        //up every single weight.

        for(int i=0; i<enc.length(); i++){
            QString e(enc.at(i));
            w += getWeight(e);
        }
    }
    if(w>1) //weights must belong to [0,1].
        w = 1;

    return w;
}

int WeightMatrix::getNumOfActiveCriteria() const
{
    mutex->lock();
    int toRet = numOfActiveCriteria;
    mutex->unlock();
    return toRet;
}

const QString WeightMatrix::getNameEncoding(QString name) const
{
    mutex->lock();
    const QString toRet = mapping->value(name);
    mutex->unlock();
    return toRet;
}

void WeightMatrix::insertCombinationWeight(QList<QString> criteriaNames, double weight)
{

    insertCombinationWeight(computeNamesEncoding(criteriaNames), weight);

}

void WeightMatrix::insertCombinationWeight(const QString &encoding, double weight)
{
    int card = encoding.length();
    mutex->lock();
    int mappingSize = mapping->size();
    mutex->unlock();

    if(card >= mappingSize)
        return;
    if(card <= 0)
        return;

    mutex->lock();
    weights->at(card-1)->insert(encoding, weight);
    mutex->unlock();
}

double WeightMatrix::getWeight(const QString &encoding) const
{
    //ldbg << "wights length = " << weights->length() << endl;
    int card = encoding.length();
    mutex->lock();
    int numActiveCrit = numOfActiveCriteria;
    mutex->unlock();
    if(card >= numActiveCrit)
        return 1;
    if(card <= 0)
        return 0;
    mutex->lock();
    double toRet = weights->at(card-1)->value(encoding);
    mutex->unlock();
    return toRet;
}

QString WeightMatrix::computeNamesEncoding(QList<QString> criteriaNames) const
{
    //ldbg << "Criteria Names: " << criteriaNames << endl;
    mutex->lock();
    if (criteriaNames.isEmpty())
            return "";
    QList<QString> enc;
    for(int i=0; i<criteriaNames.size(); i++){
        enc.append(mapping->value(criteriaNames.at(i)));
    }
    qSort(enc);
    QString toRet;
    for(int i=0; i<enc.size(); i++){
        toRet.append(enc.at(i));
    }
    mutex->unlock();
    //ldbg << "Encoding " << toRet << endl;
    return toRet;
}

QList<QString> WeightMatrix::getActiveCriteria() const
{
    mutex->lock();
    QList<QString> toRet;
    for(int i = 0; i<activeCriteria->keys().size(); i++){
        const QString k = (activeCriteria->keys().at(i));
        if(activeCriteria->value(k)){
            QString toApp = mapping->key(k);
            toRet.append(toApp);
        }
    }
    mutex->unlock();
    return toRet;
}

QList<QString> WeightMatrix::getKnownCriteria() const
{
    mutex->lock();
    QList<QString> toRet = mapping->keys();
    mutex->unlock();
    return toRet;
}

}
