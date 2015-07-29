#ifndef USERDIRECTIONCRITERION_H
#define USERDIRECTIONCRITERION_H

#include "criterion.h"

namespace Exploration{

    class UserDirectionCriterion : public Criterion
    {
    public:
        UserDirectionCriterion(double weight);
        virtual ~UserDirectionCriterion();

        double evaluate(const SLAM::Geometry::Frontier *frontier, const SLAM::Map &map,
                      int batteryTime, QHash<uint, double> &powerSignalData);

        void setAlphaX(double alphaX);
        void setAlphaY(double alphaY);

    private:
        double alphaX, alphaY;
    };
}

#endif // USERDIRECTIONCRITERION_H
