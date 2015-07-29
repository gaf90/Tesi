/*
 * topvalues.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef TOPVALUES_H_
#define TOPVALUES_H_

namespace SLAM {
namespace Support {

template <int N, typename T = int>
class TopValues
{
public:
    TopValues();
    void add(double cost, T value);
    void clear();
    int count() const;

    double cost(int i = 0) const;
    const T &value(int i = 0) const;

private:
    template<typename S>
    void shift(S *array, int start);

    double costs[N];
    T values[N];
    int maxidx;
};

template <int N, typename T>
inline TopValues<N, T>::TopValues() :
    maxidx(-1)
{
}

template <int N, typename T>
inline int TopValues<N, T>::count() const
{
    return maxidx + 1;
}

template <int N, typename T>
inline double TopValues<N, T>::cost(int i) const
{
    return costs[i];
}

template <int N, typename T>
inline const T &TopValues<N, T>::value(int i) const
{
    return values[i];
}

template <int N, typename T> template<typename S>
inline void TopValues<N, T>::shift(S *array, int start)
{
    for(int i = N - 1; i > start; i--) {
        array[i] = array[i - 1];
    }
}

template <int N, typename T>
inline void TopValues<N, T>::add(double cost, T value)
{
    int i;
    for(i = maxidx; i >= 0; i--) {
        if(cost > costs[i])
            break;
    }
    if(++i < N) {
        shift(costs, i);
        shift(values, i);
        costs[i] = cost;
        values[i] = value;
        if(maxidx < N - 1) maxidx++;
    }
}

template <int N, typename T>
inline void TopValues<N, T>::clear()
{
    maxidx = -1;
}

/* ------------------------------------ N = 1 Specialization ------------------------------------ */
/* Template specialization for a slightly more efficient minimum-only search */

template <typename T>
class TopValues<1, T>
{
public:
    inline TopValues() : first(true), cst(INFINITY) {}
    inline void add(double cost, T value) {
    	if(cost < cst) {
    		val = value;
    		cst = cost;
    		first = false;
    	}
    }
    inline int count() const { return first ? 0 : 1; }
    inline void clear() { first = true; cst = INFINITY; }

    inline double cost(int i = 0) const { Q_UNUSED(i) return cst; }
    inline const T &value(int i = 0) const { Q_UNUSED(i) return val; }

private:
    bool first;
    double cst;
    T val;
};

} /* namespace Support */
} /* namespace SLAM */

#endif /* TOPVALUES_H_ */
