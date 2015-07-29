#ifndef CRITERIONPRM_H
#define CRITERIONPRM_H

#include <QString>
#include <QHash>
#include "slam/geometry/frontier.h"
#include "slam/map.h"
#include "PRM/aStarPRM/astaralgorithmprm.h"


namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

class CriterionPRM
{
public:
    CriterionPRM();
    CriterionPRM(const QString &name, double weight, bool highGood);
    virtual ~CriterionPRM();

    const QString &getName() const;
    double getWeight() const;

    void setName(const QString &name);
    void setWeight(double weight);

    virtual double evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                    int batteryTime, QHash<uint, double> &powerSignalData) = 0;
    QList<double> getEvaluation(Frontier *point) const;
    void insertEvaluation(const Frontier *point, QList<double> value);

    void clean();
    void normalize();
    void setWorstValue(const Frontier *point);

    bool isHighGood();

private:
    void normalizeHighGood();
    void normalizeLowGood();


protected:
    QString name;
    double weight;
    double maxValue, minValue;
    bool highGood;
private:
    QHash<const Frontier *, QList<double> > evaluation;
};
}

#endif // CRITERIONPRM_H
