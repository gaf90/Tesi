#ifndef ACTION_H
#define ACTION_H

#include "pathPlanner/abstractaction.h"

namespace Data{
class Action : public PathPlanner::AbstractAction
{
public:
    enum ActionType {
            Rotation = 0,
            Translation = 1
        };

    Action(ActionType aType, double aValue);
    Action();
    virtual ~Action();

    ActionType getType() const;
    double getValue() const;

    void setType(ActionType aType);
    void setValue(double aValue);

    double getTimeEstimate();

private:
    ActionType type;
    double value;
};

}

#endif // ACTION_H
