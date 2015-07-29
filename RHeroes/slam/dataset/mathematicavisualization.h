/*
 * mathematicavisualization.h
 *
 *  Created on: 07/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef MATHEMATICAVISUALIZATION_H_
#define MATHEMATICAVISUALIZATION_H_

#include "slam/utilities.h"
#include "slam/scanmatching/retriever.h"

namespace SLAM {
namespace Dataset {

class MathematicaVisualization
{
public:
    template <typename S>
    static void associationMap(
            ScanMatching::Retriever<S> &retriever, const QList<int> associations[]);

    template <typename M, typename S>
    static void translationMap(
            const M &estimator, ScanMatching::Retriever<S> &retriever,
            const QList<int> associations[]);
};

template <typename S>
void MathematicaVisualization::associationMap(
        ScanMatching::Retriever<S> &retriever,
        const QList<int> associations[])
{
#if 0
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        fforeach(int j, associations[i]) {
            ldbg << i << " -> " << j << " " << retriever.querySegment(i) << " -> " <<
                    retriever.mapSegment(j) << " : " <<
                    ScanMatching::AssociationProbabilistic::distance(retriever.derived(), i, j) << endl;
        }
    }
#endif


    lprint << realfmt("%.6f") << "Graphics[{Red,";
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        lprint << retriever.querySegment(i) << ",";
    }
    lprint << "Null,Black,";
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            lprint << retriever.mapSegment(*it) << ",";
        }
    }
    lprint << "Null,Green,Dashed,";
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        const Geometry::LineSegment &l1 = retriever.querySegment(i);
        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            const Geometry::LineSegment &l2 = retriever.mapSegment(*it);
            Geometry::Point c1 = l1.centroid(), c2 = l2.centroid();
            Geometry::Point middle = (c1 + c2) / 2;
            Geometry::Point delta = c2 - middle;
            Geometry::Point bezierMiddle = middle + Geometry::Point(-delta.y(), delta.x());
            lprint << "BezierCurve[{" << c1 << "," << bezierMiddle << "," << c2 << "}],";
        }
    }
    lprint << "Null},Frame->True]" << endl;
}


template <typename M, typename S>
void MathematicaVisualization::translationMap(
        const M &estimator,
        ScanMatching::Retriever<S> &retriever,
        const QList<int> associations[])
{
    const Eigen::Vector2d t = estimator.pureRototranslation().template block<2,1>(0,0);
    const Geometry::Point tp(t.x(), t.y());
    ldbg << realfmt("%.6f") << "Graphics[{Red,Arrow[{{0,0}," << tp << "}],Black,";
    for(int i = 0; i < retriever.querySegmentCount(); i++) {
        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
            const Eigen::Vector2d ti = estimator.singleAssociationEstimate(i, *it).template block<2,1>(0,0);
            const Geometry::Point tpi(ti.x(), ti.y());
            ldbg << "Arrow[{{0,0}," << tpi << "}],";
        }
    }
    ldbg << "Null},Frame->True]\n";
}

} /* namespace Dataset */
} /* namespace SLAM */
#endif /* MATHEMATICAVISUALIZATION_H_ */
