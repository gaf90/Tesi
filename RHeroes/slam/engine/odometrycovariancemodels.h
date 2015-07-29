/*
 * odometrycovariancemodels.h
 *
 *  Created on: 09/gen/2013
 *      Author: Mladen Mazuran
 */

#ifndef ODOMETRYCOVARIANCEMODELS_H_
#define ODOMETRYCOVARIANCEMODELS_H_

#include "shared/config.h"
#include "slam/geometry/uncertainrototranslation.h"
#include "slam/support/mathfuncs.h"

namespace SLAM {
namespace Engine {

class BasicOdometryCovarianceModel {
public:
    static Geometry::UncertainRototranslation addCovariance(
            const Geometry::Rototranslation &deltaOdometry)
    {
        using namespace Config::SLAM;

        const double x     = std::abs(deltaOdometry.tx());
        const double y     = std::abs(deltaOdometry.ty());
        const double theta = std::abs(deltaOdometry.angle());

        /*
             Divide by three as the error constants represent the 3 sigma bounds
             (99.73% significance)
        */
        const double varX     = square(
                (maxSpatialErrorPerMeter * x + maxSpatialErrorPerRadian * theta) / 3);
        const double varY     = square(
                (maxSpatialErrorPerMeter * y + maxSpatialErrorPerRadian * theta) / 3);
        const double varTheta = square(
                (maxAngularErrorPerMeter * std::sqrt(x * x + y * y) +
                        maxAngularErrorPerRadian * theta) / 3);

        Eigen::Matrix3d cov = Eigen::Vector3d(
                max(varX, Config::SLAM::odometryMinVarianceX),
                max(varY, Config::SLAM::odometryMinVarianceY),
                max(varTheta, Config::SLAM::odometryMinVarianceTheta)).asDiagonal();

        //lprint << cov(0,0) << " " << cov(1,1) << " " << cov(2,2) << endl;

        return Geometry::UncertainRototranslation(deltaOdometry, cov);
    }
};

class ThrunOdometryCovarianceModel {
public:
    static Geometry::UncertainRototranslation addCovariance(
            const Geometry::Rototranslation &deltaOdometry)
    {
        using namespace Config::SLAM;

        const double x      = deltaOdometry.tx();
        const double y      = deltaOdometry.ty();
        const double theta  = deltaOdometry.angle();

        /* Motion is defined along the y axis, the angles are thus shifted by pi/2 */
        double transl = std::sqrt(x * x + y * y);
        const double theta1 = std::atan2(y, x) + (y > 0 ? -M_PI_2 : M_PI_2);
        const double theta2 = wrap(theta - theta1);

        /*
             Divide by three as the error constants represent the 3 sigma bounds
             (99.73% significance)
        */
        const double varTr  = square(1. / 3 *
                (maxSpatialErrorPerMeter * transl + maxSpatialErrorPerRadian * std::abs(theta)));
        const double varTh1 = square(1. / 3 *
                (maxAngularErrorPerMeter * transl + maxAngularErrorPerRadian * std::abs(theta1)));
        const double varTh2 = square(1. / 3 *
                (maxAngularErrorPerMeter * transl + maxAngularErrorPerRadian * std::abs(theta2)));

        const double c = std::cos(theta1 + M_PI_2), s = std::sin(theta1 + M_PI_2);
        Eigen::Matrix3d J;

        transl *= y > 0 ? 1 : -1;

        J <<
                c, -transl * s, 0,
                s,  transl * c, 0,
                0,           1, 1;

        Eigen::Matrix3d cov =
                J * Eigen::Vector3d(varTr, varTh1, varTh2).asDiagonal() * J.transpose();

        /* TODO: Find something more clever */
        cov(0,0) = max(cov(0,0), Config::SLAM::odometryMinVarianceX);
        cov(1,1) = max(cov(1,1), Config::SLAM::odometryMinVarianceY);
        cov(2,2) = max(cov(2,2), Config::SLAM::odometryMinVarianceTheta);

        return Geometry::UncertainRototranslation(deltaOdometry, cov);
    }
};

} /* namespace Engine */
} /* namespace SLAM */

#endif /* ODOMETRYCOVARIANCEMODELS_H_ */
