/*
 * pathnode.h
 *
 *  Created on: 24/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef PATHNODE_H_
#define PATHNODE_H_

#include "timedpose.h"
#include "slam/geometry/linesegment.h"
#include "slam/geometry/visibilitypolygon.h"

namespace SLAM {

namespace Geometry {
    class Frontier;
    //class VisibilityTriangle;
} /* namespace Geometry */

class PathNode : public TimedPose
{
public:
    PathNode();
    PathNode(double timestamp, const Data::Pose &pose);
    PathNode(const PathNode &node);
    PathNode(const TimedPose &pose);
    virtual ~PathNode();

    void setCovariance(const Eigen::Matrix3d &cov);
    bool hasCovariance() const;
    const Eigen::Matrix3d &covariance() const;

    const Geometry::VisibilityPolygon *visibility() const;
    void setVisibility(const Geometry::VisibilityPolygon *poly);

    QList<Geometry::Frontier *> &visibleFrontiers();
    const QList<Geometry::Frontier *> &visibleFrontiers() const;

    PathNode *next();
    const PathNode *next() const;
    PathNode *previous();
    const PathNode *previous() const;

    void setNext(PathNode *node);
    void setPrevious(PathNode *node);

    PathNode &operator=(const PathNode &node);

    friend LoggerStream &operator<<(LoggerStream &stream, const PathNode *node);

private:
    QList<Geometry::Frontier *> frontiers;
    PathNode *nprev, *nnext;
    Geometry::VisibilityPolygon vp;
    bool hasCov;
    Eigen::Matrix3d cov;
};

} /* namespace SLAM */

#endif /* PATHNODE_H_ */
