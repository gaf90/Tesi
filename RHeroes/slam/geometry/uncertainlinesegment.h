/*
 * uncertainlinesegment.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef UNCERTAINLINESEGMENT_H_
#define UNCERTAINLINESEGMENT_H_

#include "linesegment.h"
#include "uncertainrototranslation.h"
#include <Eigen/Core>

namespace SLAM {
namespace Geometry {

class UncertainLineSegment : public LineSegment
{
public:
    using LineSegment::VectorIndices;

    UncertainLineSegment();
    UncertainLineSegment(
            const LineSegment &ls, const Eigen::Matrix4d &cov);
    UncertainLineSegment(
            const Eigen::Vector4d &vec, const Eigen::Matrix4d &cov);
    UncertainLineSegment(
            double alpha, double rho, double psi1, double psi2, const Eigen::Matrix4d &cov);
    UncertainLineSegment(
            double alpha, double rho, const Point &p1ToProj, const Point &p2ToProj,
            const Eigen::Matrix4d &cov);
    UncertainLineSegment(const UncertainLineSegment &s);
    ~UncertainLineSegment();

    UncertainLineSegment &operator=(const UncertainLineSegment &s);

    /* Center of rotational uncertainty */
    Point cru() const;
    double angle() const;

    double alpha() const;
    double rho() const;
    double psi1() const;
    double psi2() const;

    Eigen::Vector4d vector() const;
    const Eigen::Matrix4d &covariance() const;
    Eigen::Vector2d lineVector() const;
    Eigen::Matrix2d lineCovariance() const;
    Eigen::Matrix2d p1Covariance() const;
    Eigen::Matrix2d p2Covariance() const;
    Eigen::Matrix4d transformationJacobian(const Rototranslation &rt) const;
    Eigen::Matrix<double, 4, 3> transformationJacobianRT(const Rototranslation &rt) const;

    static Point cru(double alpha, double rho, const Eigen::Matrix2d &lineCov);
    static Eigen::Matrix4d transformationJacobian(double alpha, const Rototranslation &rt);
    static Eigen::Matrix<double, 4, 3> transformationJacobianRT(
            double alpha, const Rototranslation &rt);

    double psiProjection(const Point &p) const;

    void setVector(const Eigen::Vector4d &vec);
    void setCovariance(const Eigen::Matrix4d &cov);
    void setPsi1Projection(const Point &p);
    void setPsi2Projection(const Point &p);
    void setPsi1ProjectionWithUpdate(const Point &p);
    void setPsi2ProjectionWithUpdate(const Point &p);

    void swapEndpoints();
    void updateEndpoints();
    void reorderEndpoints();

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    friend UncertainLineSegment operator*(const Rototranslation &rt,
            const UncertainLineSegment &l);
    friend UncertainLineSegment operator*(const UncertainRototranslation &rt,
            const UncertainLineSegment &l);
    friend LoggerStream &operator<<(LoggerStream &stream, const UncertainLineSegment &s);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double _alpha, _rho, _psi1, _psi2;
    Eigen::Matrix4d cov;
};

inline UncertainLineSegment::UncertainLineSegment() :
    LineSegment(), _alpha(0), _rho(0), _psi1(0), _psi2(0), cov(4,4)
{
}

inline UncertainLineSegment::UncertainLineSegment(
        const LineSegment &ls, const Eigen::Matrix4d &cov) :
        LineSegment(ls), cov(cov)
{
    const Eigen::Vector2d vec = ls.lineVector();
    _alpha = vec[AlphaVectorIndex];
    _rho = vec[RhoVectorIndex];
    setPsi1Projection(ls.p1());
    setPsi2Projection(ls.p2());
}

inline UncertainLineSegment::UncertainLineSegment(
        const Eigen::Vector4d &vec, const Eigen::Matrix4d &cov) :
    LineSegment(), _alpha(vec[AlphaVectorIndex]), _rho(vec[RhoVectorIndex]),
    _psi1(vec[Psi1VectorIndex]), _psi2(vec[Psi2VectorIndex]), cov(cov)
{
    updateEndpoints();
    //reorderPsi();
}

inline UncertainLineSegment::UncertainLineSegment(
        double alpha, double rho, double psi1, double psi2,
        const Eigen::Matrix4d &cov) :
    LineSegment(), _alpha(alpha), _rho(rho), _psi1(psi1), _psi2(psi2), cov(cov)
{
    updateEndpoints();
    //reorderPsi();
}

inline UncertainLineSegment::UncertainLineSegment(
        double alpha, double rho,
        const Point &p1ToProj, const Point &p2ToProj,
        const Eigen::Matrix4d &cov) :
    LineSegment(), _alpha(alpha), _rho(rho), cov(cov)
{
    setPsi1Projection(p1ToProj);
    setPsi2Projection(p2ToProj);
    updateEndpoints();
    //reorderPsi();
}

inline UncertainLineSegment::UncertainLineSegment(const UncertainLineSegment &s) :
    LineSegment(s), _alpha(s._alpha), _rho(s._rho), _psi1(s._psi1), _psi2(s._psi2), cov(s.cov)
{
}

inline UncertainLineSegment::~UncertainLineSegment()
{
}

inline Point UncertainLineSegment::cru() const
{
    const double ca = std::cos(_alpha), sa = std::sin(_alpha);
    const double dpsi = - cov(RhoVectorIndex, AlphaVectorIndex) /
            cov(AlphaVectorIndex, AlphaVectorIndex);
    return Point(_rho * ca + dpsi * sa, _rho * sa - dpsi * ca);
}

inline Point UncertainLineSegment::cru(double alpha, double rho, const Eigen::Matrix2d &lineCov)
{
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    const double dpsi = - lineCov(RhoVectorIndex, AlphaVectorIndex) /
            lineCov(AlphaVectorIndex, AlphaVectorIndex);
    return Point(rho * ca + dpsi * sa, rho * sa - dpsi * ca);
}

inline double UncertainLineSegment::angle() const
{
    return wrap(_alpha - M_PI_2);
}

inline double UncertainLineSegment::alpha() const
{
    return _alpha;
}

inline double UncertainLineSegment::rho() const
{
    return _rho;
}

inline double UncertainLineSegment::psi1() const
{
    return _psi1;
}

inline double UncertainLineSegment::psi2() const
{
    return _psi2;
}

inline double UncertainLineSegment::psiProjection(const Point &p) const
{
    return p.y() * std::cos(_alpha) - p.x() * std::sin(_alpha);
}

inline void UncertainLineSegment::setPsi1Projection(const Point &p)
{
    _psi1 = psiProjection(p);
}

inline void UncertainLineSegment::setPsi2Projection(const Point &p)
{
    _psi2 = psiProjection(p);
}

inline void UncertainLineSegment::setPsi1ProjectionWithUpdate(const Point &p)
{
    setPsi1Projection(p);
    updateEndpoints();
}

inline void UncertainLineSegment::setPsi2ProjectionWithUpdate(const Point &p)
{
    setPsi2Projection(p);
    updateEndpoints();
}

inline void UncertainLineSegment::swapEndpoints()
{
    std::swap(_psi1, _psi2);
    std::swap(p1v, p2v);
    cov.col(Psi1VectorIndex).swap(cov.col(Psi2VectorIndex));
    cov.row(Psi1VectorIndex).swap(cov.row(Psi2VectorIndex));
    /*
    Eigen::Matrix4d permutation;
    permutation <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 0, 1,
            0, 0, 1, 0;
    cov = (permutation * cov * permutation.transpose()).eval();
    */
}

inline void UncertainLineSegment::updateEndpoints()
{
    const double ca = std::cos(_alpha), sa = std::sin(_alpha);
    p1v = Point(_rho * ca - _psi1 * sa, _rho * sa + _psi1 * ca);
    p2v = Point(_rho * ca - _psi2 * sa, _rho * sa + _psi2 * ca);
}

inline void UncertainLineSegment::reorderEndpoints()
{
    if(_psi1 > _psi2) {
        swapEndpoints();
    }
}

inline Eigen::Vector4d UncertainLineSegment::vector() const
{
    return Eigen::Vector4d(_alpha, _rho, _psi1, _psi2);
}

inline void UncertainLineSegment::setVector(const Eigen::Vector4d &vec)
{
    _alpha = vec[AlphaVectorIndex];
    _rho   = vec[RhoVectorIndex];
    _psi1  = vec[Psi1VectorIndex];
    _psi2  = vec[Psi2VectorIndex];
}

inline const Eigen::Matrix4d &UncertainLineSegment::covariance() const
{
    return cov;
}

inline void UncertainLineSegment::setCovariance(const Eigen::Matrix4d &cov)
{
    this->cov = cov;
}

inline Eigen::Vector2d UncertainLineSegment::lineVector() const
{
    return Eigen::Vector2d(_alpha, _rho);
}

inline Eigen::Matrix2d UncertainLineSegment::lineCovariance() const
{
    return cov.block<2, 2>(0, 0);
}

inline Eigen::Matrix2d UncertainLineSegment::p1Covariance() const
{
    const double ca = std::cos(_alpha), sa = std::sin(_alpha);
    Eigen::Matrix<double, 2, 4> H;
    H <<
            -_rho * sa - _psi1 * ca, ca, -sa, 0,
             _rho * ca - _psi1 * sa, sa,  ca, 0;
    return H * cov * H.transpose();
}

inline Eigen::Matrix2d UncertainLineSegment::p2Covariance() const
{
    const double ca = std::cos(_alpha), sa = std::sin(_alpha);
    Eigen::Matrix<double, 2, 4> H;
    H <<
            -_rho * sa - _psi2 * ca, ca, 0, -sa,
             _rho * ca - _psi2 * sa, sa, 0,  ca;
    return H * cov * H.transpose();
}

inline UncertainLineSegment &UncertainLineSegment::operator=(const UncertainLineSegment &s)
{
    static_cast<LineSegment &>(*this) = s;
    _alpha = s._alpha;
    _rho   = s._rho;
    _psi1  = s._psi1;
    _psi2  = s._psi2;
    cov    = s.cov;
    return *this;
}

inline Eigen::Matrix4d UncertainLineSegment::transformationJacobian(
        const Rototranslation &rt) const
{
    return transformationJacobian(_alpha, rt);
}

inline Eigen::Matrix4d UncertainLineSegment::transformationJacobian(
        double alpha, const Rototranslation &rt)
{
    const double ca = std::cos(alpha + rt.angle()), sa = std::sin(alpha + rt.angle());
    const double drho = rt.tx() * ca + rt.ty() * sa;
    const double dpsi = rt.ty() * ca - rt.tx() * sa;
    Eigen::Matrix4d Hs;
    Hs <<
                1, 0, 0, 0,
             dpsi, 1, 0, 0,
            -drho, 0, 1, 0,
            -drho, 0, 0, 1;
    return Hs;
}

inline Eigen::Matrix<double, 4, 3> UncertainLineSegment::transformationJacobianRT(
        const Rototranslation &rt) const
{
    return transformationJacobianRT(_alpha, rt);
}


inline Eigen::Matrix<double, 4, 3> UncertainLineSegment::transformationJacobianRT(
        double alpha, const Rototranslation &rt)
{
    const double ca = std::cos(alpha + rt.angle()), sa = std::sin(alpha + rt.angle());
    const double drho = rt.tx() * ca + rt.ty() * sa;
    const double dpsi = rt.ty() * ca - rt.tx() * sa;
    Eigen::Matrix<double, 4, 3> Hr;

    Hr <<
              0,  0,     1,
             ca, sa,  dpsi,
            -sa, ca, -drho,
            -sa, ca, -drho;

    return Hr;
}

inline UncertainLineSegment operator*(
        const Rototranslation &rt, const UncertainLineSegment &l)
{
    const Eigen::Matrix4d Hs = l.transformationJacobian(rt);
    return UncertainLineSegment(
            wrap(l._alpha + rt.angle()), l._rho - Hs(2, 0),
            l._psi1 + Hs(1, 0), l._psi2 + Hs(1, 0), Hs * l.cov * Hs.transpose());
}

inline UncertainLineSegment operator*(
        const UncertainRototranslation &rt, const UncertainLineSegment &l)
{
    const double theta = rt.angle();
    const double ca = std::cos(l._alpha + theta), sa = std::sin(l._alpha + theta);

    const Eigen::Matrix4d Hs = l.transformationJacobian(rt);
    Eigen::Matrix<double, 4, 3> Hr;

    Hr <<
              0,  0,        1,
             ca, sa, Hs(1, 0), // dpsi
            -sa, ca, Hs(2, 0), // -drho
            -sa, ca, Hs(2, 0); // -drho

    return UncertainLineSegment(
            wrap(l._alpha + theta),
            l._rho - Hs(2, 0),  // rho + drho
            l._psi1 + Hs(1, 0), // psi1 + dpsi
            l._psi2 + Hs(1, 0), // psi2 + dpsi
            Hs * l.cov * Hs.transpose() + Hr * rt.covariance() * Hr.transpose());
}

} /* namespace Geometry */
} /* namespace SLAM */

#endif /* UNCERTAINLINESEGMENT_H_ */
