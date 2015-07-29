#include "abstractaction.h"

namespace PathPlanner{
    AbstractAction::AbstractAction()
    {
    }

    AbstractAction::~AbstractAction()
    {

    }

    double AbstractAction::getTimeEstimate()
    {
        return 0;
    }
}
