#ifndef POSE_H
#define POSE_H
#include "data/serializable.h"
#include "shared/logger.h"
#include <Eigen/Core>

namespace Data {

class Pose : public Serializable
{
public:
    Pose();
    Pose(double aX, double aY, double aTheta);
    Pose(const Eigen::Vector3d &vec);
    Pose(const Pose &aPose);
    virtual ~Pose();
    double getX() const;
    double getY() const;
    double getTheta() const;

    inline double x() const { return aX; }
    inline double y() const { return aY; }
    inline double theta() const { return aTheta; }
    inline Eigen::Vector2d position() const { return Eigen::Vector2d(aX, aY); }
    inline Eigen::Vector3d vectorForm() const { return Eigen::Vector3d(aX, aY, aTheta); }
    inline operator Eigen::Vector3d() const { return vectorForm(); }

    void setX(double aX);
    void setY(double aY);
    void setTheta(double aTheta);

    double getDistance(const Pose &pose) const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

    friend LoggerStream &operator<<(LoggerStream &stream, const Pose &pose);
    friend LoggerStream &operator<<(LoggerStream &stream, const Pose *pose);    
    bool operator==(const Pose &s) const;
    Pose &operator=(const Pose &pose);
    Pose &operator=(const Eigen::Vector3d &vec);

private:
    double aX, aY;
    double aTheta;

};

//bool Pose::operator==(const Pose &pose) const
//{
//	if(x == pose.x && y == pose.y && theta == pose.theta)
//		return true;
//	return false;
//}


inline uint qHash(const Pose &p) {
    union {
        double d;
        quint64 u;
    } x, y, theta;
    x.d = p.x(); y.d = p.y(); theta.d = p.theta();
    quint64 xu = x.u * 17, yu = y.u * 29, thetau = theta.u * 43;
    quint64 twoHash = xu ^ yu ^ thetau;

	return (twoHash & 0xFFFFFFFFLU) ^ (twoHash >> 32);
}

inline bool Pose::operator==(const Pose &pose) const
{
	return ((aX == pose.getX()) && (aY == pose.getY()) && (aTheta == pose.getTheta()));
}

} // namespace Data

#endif // POSE_H
