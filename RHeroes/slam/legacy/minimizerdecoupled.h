/*
 * minimizerdecoupled.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef LEGACY_MINIMIZERDECOUPLED_H_
#define LEGACY_MINIMIZERDECOUPLED_H_

#define USE_PSEUDOINVERSE

#include "slam/utilities.h"
#include "slam/geometry/segmentscan.h"
#include "shared/utilities.h"
#include <cmath>
#include <sstream>
#ifdef USE_PSEUDOINVERSE
#	include <Eigen/SVD>
#else
#	include <Eigen/Cholesky>
#endif

//#define VERBOSE_DEBUG

namespace SLAM {
namespace Legacy {

class MinimizerDecoupled
{
public:
    template <typename T>
    static Eigen::Vector3d minimize(const Geometry::SegmentScan &s,
                         const QList<T *> &map,
                         const QList<int> associations[]);
};

#ifdef VERBOSE_DEBUG
static const char *colors[] = {
        "Red", "Green", "Blue", "Black", "Gray", "Cyan", "Magenta", "Yellow", "Brown", "Orange",
        "Pink", "Purple", "LightRed", "LightGreen", "LightBlue", "LightGray", "LightCyan",
        "LightMagenta", "LightYellow", "LightBrown", "LightOrange", "LightPink", "LightPurple"
};
#endif /* VERBOSE_DEBUG */

template <typename T>
inline Eigen::Vector3d MinimizerDecoupled::minimize(
        const Geometry::SegmentScan &s,
        const QList<T *> &map,
        const QList<int> associations[])
{
    const AlignedVector<Geometry::UncertainLineSegment> &scan = s.getSegments();

    /* Find optimal angle */
    double angle = 0, sumoflengths = 0;

#ifdef VERBOSE_DEBUG
    std::stringstream ss(std::stringstream::out);
#endif /* VERBOSE_DEBUG */
    for(int i = 0; i < scan.size(); i++) {
        double segAngle = scan[i].angle();
        double segLen = scan[i].length();

#ifdef VERBOSE_DEBUG
        if(associations[i].size() > 0)
            ldbg << "AbsoluteDashing[0], " << colors[i] << ", " << scan[i] << ", Dashed, ";
#endif /* VERBOSE_DEBUG */

        for(QList<int>::const_iterator it = associations[i].begin(),
                end = associations[i].end(); it != end; ++it) {
        	int pos = *it;
        	const T &feature = *map[pos];
        	//const Geometry::UncertainLineSegmentPolar &feature = map[pos]->feature();
        	double minlen = min(feature.length(), segLen);
            //if(minlen < .2) minlen = 0;
            angle += minlen * linewrap(feature.angle() - segAngle);
            sumoflengths += minlen;

#ifdef VERBOSE_DEBUG
            ldbg << feature << ", ";
            ss << "d("<<i<<","<<pos<<") = " << AssociationElseberg<1>::distance(scan[i],feature)
                    << " " << nonOverlapDistance(scan[i],feature)
                    << " -> " << feature.angle() << " " << segAngle << " -> "
                    << linewrap(feature.angle() - segAngle) << " "
                    << minlen * linewrap(feature.angle() - segAngle) << std::endl;

#endif /* VERBOSE_DEBUG */
        }

    }
    if(almostEqual(sumoflengths, 0, 1e-9)) sumoflengths = 1;
    angle /= sumoflengths;

#ifdef VERBOSE_DEBUG
    ldbg << endl;
    ldbg << ss.str() << endl;
    ldbg << realfmt("%g") << angle << realfmt("%f") << endl;
#endif /* VERBOSE_DEBUG */

    /* Find optimal translation */
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero(), t;
    Geometry::Rototranslation rot(angle);

    for(int i = 0; i < scan.size(); i++) {
    	Eigen::Matrix2d Ai;
    	const Geometry::Point &p = rot * scan[i].p1(), q = rot * scan[i].p2();
		double px_qx = p.x() - q.x(), py_qy = p.y() - q.y();
    	double pxqy_pyqx = p.x() * q.y() - p.y() * q.x();
    	double l2 = px_qx * px_qx + py_qy * py_qy;
    	double segLen = std::sqrt(l2);
    	double sumoflengths = 0;
    	Ai <<
    			 py_qy * py_qy, -px_qx * py_qy,
    			-px_qx * py_qy,  px_qx * px_qx;

#ifdef VERBOSE_DEBUG
    	ldbg << "AbsoluteDashing[0], " << colors[i] << ", " << rot * scan[i] << ", Dashed, ";
#endif /* VERBOSE_DEBUG */

    	for(int j = 0; j < associations[i].size(); j++) {
			const T &feature = *map[associations[i][j]];
    	    //const Geometry::UncertainLineSegmentPolar &feature = map[associations[i][j]]->feature();
            const Geometry::Point &c = feature.centroid();
            double minlen = min(feature.length(), segLen);
            if(minlen < 0.2) minlen = 0;
            //if(minlen < .2) minlen = 0;
			double bshared = 2 * minlen * (c.x() * py_qy - c.y() * px_qx + pxqy_pyqx) / l2;
			b[0] -= py_qy * bshared;
			b[1] += px_qx * bshared;
			sumoflengths += minlen;

			//ldbg << "d2[{" << p << "," << q << "}," << c << ",trasl]+" << endl;
#ifdef VERBOSE_DEBUG
			ldbg << feature << ", ";
#endif /* VERBOSE_DEBUG */
		}

		A += 2 * sumoflengths * Ai / l2;
    }

#ifdef USE_PSEUDOINVERSE
    /* Solution with pseudo-inverse of A */
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d sigma = svd.singularValues();
    //ldbg << sigma << endl;
    sigma[0] = almostEqual(sigma[0], 0, 0.05) ? 0 : 1 / sigma[0];
    sigma[1] = almostEqual(sigma[1], 0, 0.05) ? 0 : 1 / sigma[1];

    t = - svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose() * b;
#else
    /* Solution with common inverse of A */
    t = A.ldlt().solve(-b);
#endif

#ifdef VERBOSE_DEBUG
    ldbg << realfmt("%g") << endl << t << realfmt("%f") << endl;
#endif /* VERBOSE_DEBUG */

    Eigen::Vector3d rt;
    rt << t, angle;

    return rt;
}

} /* namespace Legacy */
} /* namespace SLAM */

#endif /* LEGACY_MINIMIZERDECOUPLED_H_ */
