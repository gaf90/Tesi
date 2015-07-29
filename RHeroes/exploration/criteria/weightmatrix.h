#ifndef WEIGHTMATRIX_H
#define WEIGHTMATRIX_H

#include <QHash>
#include <QList>
#include <QMutex>

namespace Exploration{
    class WeightMatrix
    {
    public:
        WeightMatrix(int numOfCriteria);
        virtual ~WeightMatrix();

        void insertSingleCriterion(QString name, double weight, bool active);
        const QString getNameEncoding(QString name) const;

        void insertCombinationWeight(QList<QString> criteriaNames, double weight);
        void insertCombinationWeight(const QString &encoding, double weight);
        double getWeight(QList<QString> criteriaNames) const;
        double getWeight(const QString &encoding) const;        
        QString computeNamesEncoding(QList<QString> criteriaNames) const;
        QList<QString> getActiveCriteria() const;
        int getNumOfActiveCriteria() const;
        void changeCriteriaActivation(const QString &name, bool active);
        QList<QString> getKnownCriteria() const;
    private:

        QHash<QString, bool> *activeCriteria;
        //This member maps a criterion name with its encoding
        QHash<QString, QString> *mapping;
        //This is the double entrance table that contains all the weight.
        // - the index of the list indicate the cardinality of the weight combination.
        // - the QHash contains the pairs criteria_combination<->weight
        // NOTE: all the keys of the QString must be sorted by lexicographic order.
        QList<QHash<QString, double> * > *weights;
        int lastInsertedCriteria;
        int numOfActiveCriteria;
        QMutex *mutex;
    };
}

#endif // WEIGHTMATRIX_H
