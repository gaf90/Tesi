/*
 * alignedvector.h
 *
 *  Created on: 06/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef ALIGNEDVECTOR_H_
#define ALIGNEDVECTOR_H_

#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include "shared/loggerstream.h"

namespace SLAM {

/* QList<T>/QVector<T> clone implemented with 16-byte aligned storage */
template <typename T>
class AlignedVector {
private:
    typedef std::vector<T, Eigen::aligned_allocator<T> > storage_type;

public:
    typedef typename storage_type::iterator iterator;
    typedef typename storage_type::iterator Iterator;
    typedef T & reference;
    typedef T * pointer;
    typedef typename storage_type::const_iterator const_iterator;
    typedef typename storage_type::const_iterator ConstIterator;
    typedef const T & const_reference;
    typedef const T * const_pointer;
    typedef int size_type;
    typedef T value_type;
    typedef ptrdiff_t difference_type;

public:
    inline AlignedVector() {}
    inline AlignedVector(int size) : v(size) {}
    inline AlignedVector(int size, const T &t) : v(size, t) {}
    inline AlignedVector(const AlignedVector<T> &v) : v(v.v) {}

    inline void clear() { v.clear(); }
    inline iterator erase(iterator pos) { return v.erase(pos); }
    inline iterator erase(iterator begin, iterator end) { return v.erase(begin, end); }
    inline void insert(int i, const T &t) { v.insert(v.begin() + i, t); }
    inline iterator insert(iterator before, const T &t) { return v.insert(before, t); }
    inline void replace(int i, const T &t) { v[i] = t; }
    inline void swap(int i, int j) { std::swap(v[i], v[j]); }
    inline void move(int from, int to) { insert(to, takeAt(from)); }
    inline void resize(int size) { v.resize(size); }
    inline AlignedVector<T> &fill(const T &t, int size = -1) {
        v = storage_type(size == -1 ? v.size() : size, t); return *this; }

    inline int capacity() const { return v.capacity(); }
    inline int count(const T &t) const { return std::count(v.begin(), v.end(), t); }
    inline int count() const { return v.size(); }
    inline bool contains(const T &t) const { return std::find(v.begin(), v.end(), t) != v.end(); }
    inline int size() const { return v.size(); }
    inline int length() const { return v.size(); }
    inline bool empty() const { return v.empty(); }
    inline bool isEmpty() const { return v.empty(); }
    inline bool endsWith(const T &t) const { return v.back() == t; }
    inline bool startsWith(const T &t) const { return v.front() == t; }

    inline T &operator[](int i) { return v[i]; }
    inline T &front() { return v.front(); }
    inline T &back() { return v.back(); }
    inline T &first() { return v.front(); }
    inline T &last() { return v.back(); }

    inline const T &at(int i) const { return v[i]; }
    inline const T &operator[](int i) const { return v[i]; }
    inline const T &front() const { return v.front(); }
    inline const T &back() const { return v.back(); }
    inline const T &first() const { return v.front(); }
    inline const T &last() const { return v.back(); }

    inline T value(int i) const { if(i >= 0 && i < size()) { return v[i]; } else { return T(); } }
    inline T value(int i, const T &defaultValue) const {
        if(i >= 0 && i < size()) { return v[i]; } else { return defaultValue; } }

    inline iterator begin() { return v.begin(); }
    inline iterator end() { return v.end(); }
    inline const_iterator begin() const { return v.begin(); }
    inline const_iterator end() const { return v.end(); }
    inline const_iterator constBegin() const { return v.begin(); }
    inline const_iterator constEnd() const { return v.end(); }

    inline void pop_back() { v.pop_back(); }
    inline void pop_front() { v.erase(v.begin()); }
    inline void push_back(const T &t) { v.push_back(t); }
    inline void push_front(const T &t) { v.insert(v.begin(), t); }
    inline void append(const T &t) { v.push_back(t); }
    inline void prepend(const T &t) { v.insert(v.begin(), t); }

    inline void removeAt(int i) { v.erase(v.begin() + i); }
    inline void removeFirst() { v.erase(v.begin()); }
    inline void removeLast() { v.pop_back(); }

    inline T takeAt(int i) { T ret = v[i]; removeAt(i); return ret; }
    inline T takeFirst() { T ret = first(); removeFirst(); return ret; }
    inline T takeLast() { T ret = last(); removeLast(); return ret; }

    inline void reserve(int alloc) { v.reserve(alloc); }

    inline AlignedVector<T> &operator=(const AlignedVector<T> &other) { v = other.v; return *this; }
    inline AlignedVector<T> &operator<<(const T &t) { return *this += t; }
    inline AlignedVector<T> &operator<<(const AlignedVector<T> &other) { return *this += other; }
    inline AlignedVector<T> &operator+=(const T &t) { v.push_back(t); return *this; }
    inline AlignedVector<T> &operator+=(const AlignedVector<T> &other) {
        append(other); return *this; }
    inline AlignedVector<T> operator+(const AlignedVector<T> &v) const {
        AlignedVector<T> ret = *this; ret.append(v); return ret; }
    inline bool operator!=(const AlignedVector<T> &other) const { return !(*this == other); }
    inline bool operator==(const AlignedVector<T> &other) const {
        if(size() != other.size()) return false;
        return std::equal(begin(), end(), other.begin());
    }

    inline void append(const AlignedVector<T> &v) {
        reserve(size() + v.size());
        for(const_iterator it = v.begin(); it != v.end(); it++) {
            this->v.push_back(*it);
        }
    }
    inline int indexOf(const T &t, int from = 0) const {
        const_iterator it = v.begin() + from;
        for(int i = from; i < size(); i++, it++) {
            if(*it == t) return i;
        }
        return -1;
    }
    inline int lastIndexOf(const T &t, int from = -1) const {
        if(from == -1) from = size() - 1;
        from = size() - from - 1;
        typename storage_type::const_reverse_iterator it = v.rbegin() + from;
        for(int i = from; i < size(); i++, it++) {
            if(*it == t) return size() - i - 1;
        }
        return -1;
    }
    inline int removeAll(const T &t) {
        int i = 0;
        iterator it = v.begin();
        while((it = std::find(it, v.end(), t)) != v.end()) {
            it = v.erase(it);
            i++;
        }
        return i;
    }
    inline bool removeOne(const T &t) {
        iterator it = std::find(v.begin(), v.end(), t);
        if(it != v.end()) {
            v.erase(it);
            return true;
        } else {
            return false;
        }
    }
    inline AlignedVector<T> mid(int pos, int length = -1) {
        AlignedVector<T> ret;
        if(length == -1) length = size() - pos;
        ret.v = storage_type(v.begin() + pos, v.begin() + (pos + length));
        return ret;
    }

#if __cplusplus > 199711L
    // Missing (requires C++11)
    const T *constData() const { return v.data(); }
    const T *data() const { return v.data(); }
    T *data() { return v.data(); }
    void squeeze() { return v.shrink_to_fit(); }
#endif

private:
    storage_type v;
};

template <typename T>
LoggerStream &operator<<(LoggerStream &stream, const AlignedVector<T> &v)
{
    return stream.sharedListOperator(v.begin(), v.end());
}

} /* namespace SLAM */

#endif /* ALIGNEDVECTOR_H_ */
