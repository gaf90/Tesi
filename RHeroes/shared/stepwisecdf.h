/*
 * stepwisecdf.h
 *
 *  Created on: 06/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef STEPWISECDF_H_
#define STEPWISECDF_H_

#include <QList>
#include <QVector>

namespace Shared {

/* Stepwise cumulative distribution function */
template <typename T>
class StepwiseCDF {
public:
    StepwiseCDF();
    StepwiseCDF(const QList<double> &densities, const QList<T> &values);
    StepwiseCDF(const StepwiseCDF<T> &cdf);
    StepwiseCDF &operator=(const StepwiseCDF<T> &cdf);
    int stepCount() const;
    const T &operator[](int i) const;

    T inverseLookup(double p) const;
private:
    QVector<double> weights;
    QVector<T> values;
};

template <typename T>
StepwiseCDF<T>::StepwiseCDF()
{
}

template <typename T>
StepwiseCDF<T>::StepwiseCDF(const QList<double> &densities, const QList<T> &values) :
    weights(densities.size()), values(densities.size())
{
    weights[0] = densities[0];
    /* Build cumulative vector */
    for(int i = 1; i < weights.size(); i++) {
        weights[i] = weights[i - 1] + densities[i];
    }

    /* Normalize */
    for(int i = 0; i < weights.size(); i++) {
        weights[i] /= weights.last();
        this->values[i] = values[i];
    }
}

template <typename T>
StepwiseCDF<T>::StepwiseCDF(const StepwiseCDF<T> &cdf)
{
    *this = cdf;
}

template <typename T>
StepwiseCDF<T> &StepwiseCDF<T>::operator=(const StepwiseCDF<T> &cdf)
{
    weights = cdf.weights;
    values = cdf.values;
    return *this;
}

template <typename T>
int StepwiseCDF<T>::stepCount() const
{
    return weights.size();
}

template <typename T>
const T &StepwiseCDF<T>::operator[](int i) const
{
    return values[i];
}

template <typename T>
T StepwiseCDF<T>::inverseLookup(double p) const
{
    /* THETA(log2(N)) binary search lookup */
    int start = 0, end = weights.size() - 1;
    while(end - start > 1) {
        int middle = (start + end) / 2;
        if(p < weights[middle]) {
            end = middle;
        } else {
            start = middle;
        }
    }
    if(p < weights[start])
        end = start;
    return values[end];
}

} /* namespace Shared */

#endif /* STEPWISECDF_H_ */
