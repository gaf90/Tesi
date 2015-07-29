/*
 * scanmatcher.h
 *
 *  Created on: 06/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef SCANMATCHER_H_
#define SCANMATCHER_H_

#include "slam/geometry/rototranslation.h"
#include "retriever.h"
#include <QList>
#include <Eigen/Core>

namespace SLAM {
namespace ScanMatching {

template <typename S>
class ScanMatcher {
public:
    ScanMatcher();
    virtual ~ScanMatcher();

    void setRetriever(Retriever<S> &retriever);
	virtual bool run() = 0;

	const Eigen::Vector3d &measure() const;
	const QList<int> *associations() const;
	QList<int> *associations(bool yield = false);
	int associationCount() const;
	int distinctAssociationCount() const;

	virtual Eigen::Matrix3d covariance() = 0;

protected:
    Retriever<S> *retriever;
	Eigen::Vector3d rt;
    QList<int> *assoc;
    bool owner;
};

template <typename S>
ScanMatcher<S>::ScanMatcher() :
    retriever(NULL), rt(0,0,0), assoc(NULL), owner(true)
{
}

template <typename S>
ScanMatcher<S>::~ScanMatcher()
{
    if(owner)
        delete[] assoc;
}

template <typename S>
void ScanMatcher<S>::setRetriever(Retriever<S> &retriever)
{
    this->retriever = &retriever;
    delete[] assoc;
    assoc = new QList<int>[retriever.querySegmentCount()];
}

template <typename S>
const Eigen::Vector3d &ScanMatcher<S>::measure() const
{
    return rt;
}

template <typename S>
const QList<int> *ScanMatcher<S>::associations() const
{
    return assoc;
}

template <typename S>
QList<int> *ScanMatcher<S>::associations(bool yield)
{
    owner = owner && !yield;
    return assoc;
}

template <typename S>
int ScanMatcher<S>::associationCount() const
{
    int count = 0;
    for(int i = 0; i < retriever->querySegmentCount(); i++) {
        count += assoc[i].size();
    }
    return count;
}

template <typename S>
int ScanMatcher<S>::distinctAssociationCount() const
{
    int count = 0;
    for(int i = 0; i < retriever->querySegmentCount(); i++) {
        count += assoc[i].empty() ? 0 : 1;
    }
    return count;
}

} /* namespace ScanMatching */
} /* namespace SLAM */
#endif /* SCANMATCHER_H_ */
