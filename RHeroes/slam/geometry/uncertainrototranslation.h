/*
 * uncertainrototranslation.h
 *
 *  Created on: 05/gen/2012
 *      Author: Mladen Mazuran
 */

#ifndef UNCERTAINROTOTRANSLATION_H_
#define UNCERTAINROTOTRANSLATION_H_

#include "rototranslation.h"

namespace SLAM {
namespace Geometry {

class UncertainRototranslation : public Rototranslation
{
public:
    UncertainRototranslation();
    UncertainRototranslation(const Rototranslation &rt, const Eigen::Matrix3d &cov);
    UncertainRototranslation(const Data::Pose &pose, const Eigen::Matrix3d &cov);

    UncertainRototranslation operator*(const UncertainRototranslation &rt) const;
    UncertainRototranslation &operator=(const UncertainRototranslation &rt);
    UncertainRototranslation inverse() const;
    const Eigen::Matrix3d &covariance() const;

    friend UncertainRototranslation operator*(
            const Rototranslation &rt1, const UncertainRototranslation &rt2);
    friend UncertainRototranslation operator*(
            const UncertainRototranslation &rt1, const Rototranslation &rt2);

private:
    Eigen::Matrix3d cov;
};

inline UncertainRototranslation::UncertainRototranslation() :
        Rototranslation(), cov(Eigen::Matrix3d::Identity())
{
}

inline UncertainRototranslation::UncertainRototranslation(
        const Rototranslation &rt, const Eigen::Matrix3d &cov) :
    Rototranslation(rt), cov(cov)
{
}

inline UncertainRototranslation::UncertainRototranslation(
        const Data::Pose &pose, const Eigen::Matrix3d &cov) :
    Rototranslation(pose), cov(cov)
{
}

inline const Eigen::Matrix3d &UncertainRototranslation::covariance() const
{
    return cov;
}

inline UncertainRototranslation UncertainRototranslation::inverse() const
{
    Eigen::Matrix3d J;
    J <<
            -m11, -m21, m13 * m21 - m23 * m11, // -cos(t), -sin(t), x * sin(t) - y * cos(t)
             m21, -m11, m13 * m11 + m23 * m21, //  sin(t), -cos(t), x * cos(t) + y * sin(t)
               0,    0,                    -1;

    return UncertainRototranslation(Rototranslation::inverse(), J * cov * J.transpose());
}

inline UncertainRototranslation UncertainRototranslation::operator*(
        const UncertainRototranslation &rt) const
{
    Eigen::Matrix3d J1, J2;

    J1 <<
            1, 0, -rt.m13 * m21 - rt.m23 * m11, // 1, 0, -x2 * sin(t1) - y2 * cos(t1)
            0, 1,  rt.m13 * m11 - rt.m23 * m21, // 0, 1,  x2 * cos(t1) - y2 * sin(t1)
            0, 0,                           1;
    J2 <<
            m11, -m21, 0, // cos(t1), -sin(t1), 0
            m21,  m11, 0, // sin(t1),  cos(t1), 0
              0,    0, 1;

    return UncertainRototranslation(
            static_cast<const Rototranslation &>(*this) * static_cast<const Rototranslation &>(rt),
            J1 * cov * J1.transpose() + J2 * rt.cov * J2.transpose());
}

inline UncertainRototranslation &UncertainRototranslation::operator=(
        const UncertainRototranslation &rt)
{
    static_cast<Rototranslation &>(*this) = rt;
    cov = rt.cov;
    return *this;
}

inline UncertainRototranslation operator*(
        const Rototranslation &rt1, const UncertainRototranslation &rt2)
{
    Eigen::Matrix3d J;
    J <<
            rt1.cosAngle(), -rt1.sinAngle(), 0,
            rt1.sinAngle(),  rt1.cosAngle(), 0,
                         0,               0, 1;
    return UncertainRototranslation(
                rt1 * static_cast<const Rototranslation &>(rt2), J * rt2.cov * J.transpose());
}

inline UncertainRototranslation operator*(
        const UncertainRototranslation &rt1, const Rototranslation &rt2)
{
    Eigen::Matrix3d J;

    J <<
            1, 0, -rt2.tx() * rt1.m21 - rt2.ty() * rt1.m11, // 1, 0, -x2 * sin(t1) - y2 * cos(t1)
            0, 1,  rt2.tx() * rt1.m11 - rt2.ty() * rt1.m21, // 0, 1,  x2 * cos(t1) - y2 * sin(t1)
            0, 0,  1;

    return UncertainRototranslation(
                static_cast<const Rototranslation &>(rt1) * rt2, J * rt1.cov * J.transpose());
}

// Mathematica code for UncertainPose
/*
 UncertainPose[{x_, y_, t_}, cov_] :=
  Module[{chol, tlim1, tlim2, orient, a, st, lorient, l1, l2},
   chol = Transpose[CholeskyDecomposition[cov[[1 ;; 2, 1 ;; 2]]]];
   a = t + Pi/2;
   st = 3 Sqrt[cov[[3, 3]]];
   orient = {Cos[a], Sin[a]};
   tlim1 = {Cos[a + st], Sin[a + st]};
   tlim2 = {Cos[a - st], Sin[a - st]};
   lorient =
    3/Sqrt[({orient}.Inverse[
          cov[[1 ;; 2, 1 ;; 2]]].Transpose[{orient}])[[1, 1]]];
   l1 = 3/
     Sqrt[({tlim1}.Inverse[
          cov[[1 ;; 2, 1 ;; 2]]].Transpose[{tlim1}])[[1, 1]]];
   l2 = 3/
     Sqrt[({tlim2}.Inverse[
          cov[[1 ;; 2, 1 ;; 2]]].Transpose[{tlim2}])[[1, 1]]];
   Return[{GeometricTransformation[Circle[{0, 0}, 3],
     AffineTransform[{chol, {x, y}}]],
    Arrow[{{x, y}, lorient orient + {x, y}}],
    Line[{{x, y}, l1 tlim1 + {x, y}}],
    Line[{{x, y}, l2 tlim2 + {x, y}}]}];
   ];
*/

inline LoggerStream &operator<<(LoggerStream &stream, const UncertainRototranslation &urt)
{
    return stream << "UncertainPose[{" << urt.tx() << "," << urt.ty() << "," <<
            urt.angle() << "}," << urt.covariance() << "]";
}


} /* namespace Geometry */
} /* namespace SLAM */

#endif /* UNCERTAINROTOTRANSLATION_H_ */
