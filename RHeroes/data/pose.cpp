#include "pose.h"
#include <cmath>

namespace Data {

Pose::Pose():
    aX(0), aY(0), aTheta(0)
{
}

Pose::Pose(double aX, double aY, double aTheta):
    aX(aX), aY(aY), aTheta(aTheta)
{
}

Pose::Pose(const Eigen::Vector3d &vec):
    aX(vec[0]), aY(vec[1]), aTheta(vec[2])
{
}

Pose::Pose(const Pose &aPose) :
    Serializable(), aX(aPose.x()), aY(aPose.y()), aTheta(aPose.theta())
{

}

Pose::~Pose(){

}

double Pose::getX() const
{
    return aX;
}

double Pose::getY() const
{
    return aY;
}

double Pose::getTheta() const
{
    return aTheta;
}

void Pose::setX(double aX)
{
    this->aX = aX;
}

void Pose::setY(double aY)
{
    this->aY = aY;
}

double Pose::getDistance(const Pose &pose) const
{
    return std::sqrt((pose.aX - aX) * (pose.aX - aX) + (pose.aY - aY) * (pose.aY - aY));
}

void Pose::setTheta(double aTheta)
{
    this->aTheta = aTheta;
}

void Pose::serializeTo(QDataStream &stream) const
{
    stream << aX << aY << aTheta;
}

void Pose::deserializeFrom(QDataStream &stream)
{
    stream >> aX >> aY >> aTheta;
}

Pose &Pose::operator=(const Pose &pose)
{
	aX = pose.aX;
	aY = pose.aY;
	aTheta = pose.aTheta;
	return *this;
}

Pose &Pose::operator=(const Eigen::Vector3d &vec)
{
    aX = vec[0];
    aY = vec[1];
    aTheta = vec[2];
    return *this;
}

LoggerStream &operator<<(LoggerStream &stream, const Pose &pose)
{
    return stream << "{" << pose.aX << "," << pose.aY << "," << pose.aTheta << "}";
}

LoggerStream &operator<<(LoggerStream &stream, const Pose *pose)
{
    return stream << *pose;
}

}

