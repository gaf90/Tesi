#ifndef CRITERION_H
#define CRITERION_H

#include <QString>
#include <QHash>
#include "slam/geometry/frontier.h"
#include "slam/map.h"


namespace Exploration{
class Criterion
{
public:
    Criterion();
    Criterion(const QString &name, double weight, bool highGood);
    virtual ~Criterion();

    const QString &getName() const;
    double getWeight() const;

    void setName(const QString &name);
    void setWeight(double weight);

    virtual double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData) = 0;
    double getEvaluation(SLAM::Geometry::Frontier *point) const;
    void insertEvaluation(const SLAM::Geometry::Frontier *point, double value);

    void clean();
    void normalize();
    void setWorstValue(SLAM::Geometry::Frontier *point);

private:
    void normalizeHighGood();
    void normalizeLowGood();


protected:
    QString name;
    double weight;
    double maxValue, minValue;
    bool highGood;
private:
    QHash<const SLAM::Geometry::Frontier *, double> evaluation;

};
}

#endif // CRITERION_H
