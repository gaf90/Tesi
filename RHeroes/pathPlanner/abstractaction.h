#ifndef ABSTRACTACTION_H
#define ABSTRACTACTION_H

namespace PathPlanner{
class AbstractAction
{
public:
    AbstractAction(); // {}
    virtual ~AbstractAction(); // {}

    virtual double getTimeEstimate(); //{return 0.0;}
};

}

#endif // ABSTRACTACTION_H
