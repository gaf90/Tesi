/*
 * uncertainlinesegment.cpp
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#include "uncertainlinesegment.h"

namespace SLAM {
namespace Geometry {

QDataStream &operator<<(QDataStream &stream, const Eigen::Matrix4d &mat)
{
    for(int i = 0; i < mat.rows(); i++) {
        for(int j = 0; j < mat.cols(); j++) {
            stream << mat(i, j);
        }
    }
    return stream;
}

QDataStream &operator>>(QDataStream &stream, Eigen::Matrix4d &mat)
{
    for(int i = 0; i < mat.rows(); i++) {
        for(int j = 0; j < mat.cols(); j++) {
            double d;
            stream >> d;
            mat(i, j) = d;
        }
    }
    return stream;
}

void UncertainLineSegment::serializeTo(QDataStream &stream) const
{
    stream << _rho << _alpha << _psi1 << _psi2 << cov;
}

void UncertainLineSegment::deserializeFrom(QDataStream &stream)
{
    stream >> _rho >> _alpha >> _psi1 >> _psi2 >> cov;
    updateEndpoints();
}

/*
    Mathematica code required for visualization. Set the variable SigmaSignificance to the
    significance value to visualize.

    PointAtRhoPsi[l_, {r_, p_}] :=
       {{Cos[l[[1, 1]]], -Sin[l[[1, 1]]]}, {Sin[l[[1, 1]]],
         Cos[l[[1, 1]]]}}.{r, p};
    PointAtPsi[l_, p_] := PointAtRhoPsi[l, {l[[1, 2]], p}];
    Endpoint1[l_] := PointAtPsi[l, l[[1, 3]]];
    Endpoint2[l_] := PointAtPsi[l, l[[1, 4]]];
    CRU[l_] := PointAtPsi[l, l[[2, 1, 2]]/l[[2, 1, 1]]];
    SigmaRhoAtPsi[{{a_, r_, p1_, p2_}, cov_}, p_] :=
      Sqrt[cov[[2, 2]] + p (-2 cov[[1, 2]] + p cov[[1, 1]])];
    UncertainLine[l_] :=
      Module[{a, r, p1, p2, cov, plot1, plot2, pos1, pos2, arg1, arg2},
       {{a, r, p1, p2}, cov} = l;
       plot1 =
        ParametricPlot[
         PointAtRhoPsi[
          l, {r + SigmaSignificance SigmaRhoAtPsi[l, p], p}], {p,
          l[[1, 3]], l[[1, 4]]}];
       plot2 =
        ParametricPlot[
         PointAtRhoPsi[
          l, {r - SigmaSignificance SigmaRhoAtPsi[l, p], p}], {p,
          l[[1, 3]], l[[1, 4]]}];
       pos1 = Position[plot1, Line];
       pos2 = Position[plot2, Line];
       pos1[[1, -1]] = 1;
       pos2[[1, -1]] = 1;
       arg1 = Extract[plot1, pos1];
       arg2 = Extract[plot2, pos2];
       {Line[{Endpoint1[l], Endpoint2[l]}], Disk[CRU[l], 0.01], Green,
        Line[arg1], Line[arg2], EdgeForm[], Opacity[0.2], FaceForm[Green],
         Polygon[Join[arg1[[1]], Reverse[arg2[[1]]]]]}
       ];
*/
LoggerStream &operator<<(LoggerStream &stream, const UncertainLineSegment &s)
{
    return stream << "UncertainLine[{{" <<
                s.alpha() << "," << s.rho() << "," << s.psi1() << "," << s.psi2() << "}," <<
                s.covariance() << "}]";
}

} /* namespace Geometry */
} /* namespace SLAM */
