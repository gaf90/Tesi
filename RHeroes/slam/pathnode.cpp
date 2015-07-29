/*
 * pathnode.cpp
 *
 *  Created on: 24/mar/2012
 *      Author: Mladen Mazuran
 */

#include "pathnode.h"
#include "slam/geometry/frontier.h"

namespace SLAM {

PathNode::PathNode() : TimedPose(), nprev(NULL), nnext(NULL), hasCov(false)
{
}

PathNode::PathNode(const PathNode &node) :
    TimedPose(node), nprev(node.nprev), nnext(node.nnext), vp(node.vp), hasCov(node.hasCov),
    cov(node.cov)
{
}

PathNode::PathNode(double timestamp, const Data::Pose &pose) :
    TimedPose(timestamp, pose), nprev(NULL), nnext(NULL), hasCov(false)
{
}

PathNode::PathNode(const TimedPose &pose) :
    TimedPose(pose), nprev(NULL), nnext(NULL), hasCov(false)
{
}

PathNode::~PathNode()
{
}

const Geometry::VisibilityPolygon *PathNode::visibility() const
{
    return &vp;
}

void PathNode::setVisibility(const Geometry::VisibilityPolygon *poly)
{
    vp = *poly;
}

QList<Geometry::Frontier *> &PathNode::visibleFrontiers()
{
    return frontiers;
}

const QList<Geometry::Frontier *> &PathNode::visibleFrontiers() const
{
    return frontiers;
}

PathNode *PathNode::next()
{
    return nnext;
}

const PathNode *PathNode::next() const
{
    return nnext;
}

PathNode *PathNode::previous()
{
    return nprev;
}

const PathNode *PathNode::previous() const
{
    return nprev;
}

void PathNode::setNext(PathNode *node)
{
    nnext = node;
}

void PathNode::setPrevious(PathNode *node)
{
    nprev = node;
}

void PathNode::setCovariance(const Eigen::Matrix3d &cov)
{
    hasCov = true;
    this->cov = cov;
}

bool PathNode::hasCovariance() const
{
    return hasCov;
}

const Eigen::Matrix3d &PathNode::covariance() const
{
    return cov;
}

PathNode &PathNode::operator=(const PathNode &node)
{
	(*this) = static_cast<const TimedPose &>(node);
	hasCov = node.hasCov;
	cov = node.cov;
	return *this;
}

LoggerStream &operator<<(LoggerStream &stream, const PathNode *node)
{
    return stream << *node;
}

} /* namespace SLAM */
