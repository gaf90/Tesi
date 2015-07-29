/*
 * indexset.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef INDEXSET_H
#define INDEXSET_H

#include <QList>
#include <QtAlgorithms>
#include <utility>
#include "fforeach.h"

namespace SLAM {
namespace Support {

class IndexSet
{
public:
    IndexSet();
    IndexSet(int b, int e);
    IndexSet(const QList<int> &values);
    virtual ~IndexSet();

    void add(int i);

    class const_iterator;

    const_iterator begin() const;
    const_iterator end() const;
    int indexBegin() const;
    int indexEnd() const;
    int size() const;
    int operator[](int i) const;
    QList<int> toList() const;

    const IndexSet firstHalfSplit(int midpoint) const;
    const IndexSet secondHalfSplit(int midpoint) const;

    const IndexSet join(const IndexSet &second) const;

    friend LoggerStream &operator<<(LoggerStream &stream, const IndexSet &set);

private:
    typedef std::pair<int, int> Portion;

    void addPortion(const Portion &p);
    int portionLookup(int value) const;

private:
    QList<Portion> portions;
    QList<int> lookups;
    int s;
};

class IndexSet::const_iterator {
public:
   const_iterator(const IndexSet *s);
   const_iterator(const IndexSet *s, int portion, int i);
   const_iterator(const const_iterator &c);
   const_iterator &operator++();

   int operator*();
   const_iterator &operator=(const const_iterator &c);
   bool operator==(const const_iterator &c);
   bool operator!=(const const_iterator &c);
   static const_iterator begin(const IndexSet *s);
   static const_iterator end(const IndexSet *s);

private:
   const IndexSet *s;
   int portion, i;
};

/* ========================================================================== */
/*                          IndexSet implementation                           */
/* ========================================================================== */

inline IndexSet::IndexSet(int b, int e) : s(e - b) {
    portions.append(std::make_pair(b, e));
    lookups.append(e - b);
}

inline IndexSet::IndexSet(const QList<int> &values) : s(0) {
    QList<int>::const_iterator it = values.begin(), end = values.end();
    int start = *it, previous = *it;
    ++it;
    while(it != end) {
        if(previous + 1 != *it) {
            addPortion(std::make_pair(start, previous + 1));
            start = *it;
            previous = start;
        } else {
            previous++;
        }
        ++it;
    }
    if(start != previous) {
        addPortion(std::make_pair(start, previous + 1));
    }
}

inline IndexSet::IndexSet() : s(0) {}

inline IndexSet::~IndexSet() {}

inline IndexSet::const_iterator IndexSet::begin() const {
    return const_iterator::begin(this);
}

inline IndexSet::const_iterator IndexSet::end() const {
    return const_iterator::end(this);
}

inline int IndexSet::indexBegin() const {
    return portions.first().first;
}

inline int IndexSet::indexEnd() const {
    return portions.last().second;
}

inline int IndexSet::size() const {
    return s;
}

inline void IndexSet::add(int i) {
    if(s > 0 && portions.last().second == i) {
        portions.last().second++;
        lookups.last()++;
        s++;
    } else {
       addPortion(std::make_pair(i, i + 1));
    }
}

inline void IndexSet::addPortion(const Portion &p) {
    if(p.first == p.second) return;
    portions.append(p);
    s += p.second - p.first;
    lookups.append(s);
}

inline int IndexSet::portionLookup(int value) const {
    int i = 0;
    fforeach(const Portion &p, portions) {
        if(value < p.second)
            return i;
        i++;
    }
    return i - 1;
}

// TODO: Sbagliato
inline int IndexSet::operator[](int i) const {
    return *qLowerBound(lookups.begin(), lookups.end(), i);
}

inline QList<int> IndexSet::toList() const {
    QList<int> ret;
    fforeach(int i, *this) {
        ret << i;
    }
    return ret;
}

inline const IndexSet IndexSet::firstHalfSplit(int midpoint) const {
    IndexSet is;
    int i = 0, portion = portionLookup(midpoint);
    is.s = 0;
    while(i < portion) {
        is.addPortion(portions[i]);
        i++;
    }
    if(portions[portion].first != midpoint + 1)
        is.addPortion(std::make_pair(portions[portion].first, midpoint + 1));
    return is;
}

inline const IndexSet IndexSet::secondHalfSplit(int midpoint) const {
    IndexSet is;
    int portion = portionLookup(midpoint);
    int i = portion + 1;
    is.s = 0;
    if(midpoint != portions[portion].second)
        is.addPortion(std::make_pair(midpoint, portions[portion].second));
    while(i < portions.size()) {
        is.addPortion(portions[i]);
        i++;
    }
    return is;
}

inline const IndexSet IndexSet::join(const IndexSet &second) const {
    IndexSet is;
    QList<Portion>::const_iterator it1  = portions.begin(), it2  = second.portions.begin();
    QList<Portion>::const_iterator end1 = portions.end(),   end2 = second.portions.end();

    is.s = 0;
    while(it1 != end1 && it2 != end2) {
        const Portion &p1 = *it1, &p2 = *it2;
        /*
            TODO: a portion overlapping multiple portions is not handled, although with the way
            SegmentScan is implemented it should never happen.
        */
        if(p1.second < p2.first) {
            is.addPortion(p1);
            ++it1;
        } else if(p2.second < p1.first) {
            is.addPortion(p2);
            ++it2;
        } else if(p1.second == p2.first) {
            is.addPortion(std::make_pair(p1.first, p2.second));
            ++it1; ++it2;
        } else if(p2.second == p1.first) {
            is.addPortion(std::make_pair(p2.first, p1.second));
            ++it1; ++it2;
        } else if(p1.second > p2.first && p1.second < p2.second) {
            is.addPortion(std::make_pair(p1.first, p2.second));
            ++it1; ++it2;
        } else {
            is.addPortion(std::make_pair(p2.first, p1.second));
            ++it1; ++it2;
        }
    }

    for(; it1 != end1; ++it1) {
        is.addPortion(*it1);
    }
    for(; it2 != end2; ++it2) {
        is.addPortion(*it2);
    }

    return is;
}

inline LoggerStream &operator<<(LoggerStream &stream, const IndexSet &set) {
    return stream << set.portions;
}

/* ========================================================================== */
/*                 IndexSet::const_iterator implementation                    */
/* ========================================================================== */

inline IndexSet::const_iterator::const_iterator(const IndexSet *s) :
    s(s), portion(0), i(s->indexBegin()) {}

inline IndexSet::const_iterator::const_iterator(const IndexSet *s, int portion, int i) :
    s(s), portion(portion), i(i) {}

inline IndexSet::const_iterator::const_iterator(const const_iterator &c) :
    s(c.s), portion(c.portion), i(c.i) {}

inline IndexSet::const_iterator &IndexSet::const_iterator::operator++() {
    i++;
    if(i == s->portions[portion].second) {
        portion++;
        if(portion < s->portions.size()) {
            i = s->portions[portion].first;
        } else {
            i = -1;
        }
    }
    return *this;
}

inline int IndexSet::const_iterator::operator*() {
    return i;
}

inline IndexSet::const_iterator &IndexSet::const_iterator::operator=(const const_iterator &c) {
    s = c.s;
    portion = c.portion;
    i = c.i;
    return *this;
}

inline bool IndexSet::const_iterator::operator==(
    const IndexSet::const_iterator &c) {
    return i == c.i;
}

inline bool IndexSet::const_iterator::operator!=(
    const IndexSet::const_iterator &c) {
    return i != c.i;
}

inline IndexSet::const_iterator
IndexSet::const_iterator::begin(const IndexSet *s) {
    return IndexSet::const_iterator(s);
}

inline IndexSet::const_iterator
IndexSet::const_iterator::end(const IndexSet *s) {
    return IndexSet::const_iterator(s, 0, -1);
}

} /* namespace Support */
} /* namespace SLAM */

#endif /* INDEXSET_H */
