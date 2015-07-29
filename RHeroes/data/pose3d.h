#ifndef POSE3D_H
#define POSE3D_H
#include "data/serializable.h"
#include "shared/utilities.h"
#include "shared/logger.h"
#include <Eigen/Core>

namespace Data {

class Pose3D : public Serializable
{
public:
    inline Pose3D() : aX(0), aY(0), aZ(0), aTheta(0), aPhi(0), aPsi(0) {}
    inline Pose3D(double aX, double aY, double aZ, double aTheta, double aPhi, double aPsi) :
        aX(aX), aY(aY), aZ(aZ), aTheta(wrapRad(aTheta)), aPhi(wrapRad(aPhi)), aPsi(wrapRad(aPsi)) {}
    virtual ~Pose3D() {}

    inline double x() const { return aX; }
    inline double y() const { return aY; }
    inline double z() const { return aZ; }
    inline double theta() const { return aTheta; }
    inline double phi() const { return aPhi; }
    inline double psi() const { return aPsi; }

    virtual void serializeTo(QDataStream &stream) const { stream << aX << aY << aZ << aTheta << aPhi << aPsi; }
    virtual void deserializeFrom(QDataStream &stream) { stream >> aX >> aY >> aZ >> aTheta >> aPhi >> aPsi; }
    friend LoggerStream &operator<<(LoggerStream &stream, const Pose3D &p);

private:
    double aX, aY, aZ;
    double aTheta, aPhi, aPsi;

};

inline LoggerStream &operator<<(LoggerStream &stream, const Pose3D &p)
{
    return stream << "{" << p.x() << "," << p.y() << "," << p.z() << ","
                  << p.theta() << ","  << p.phi() << "," << p.psi() << "}";
}

} // namespace Data

#endif // POSE3D_H
