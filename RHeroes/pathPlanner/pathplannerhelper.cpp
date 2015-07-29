#include "pathplannerhelper.h"

#include <QApplication>
#include "data/buddymessage.h"
#include "data/wirelessmessage.h"
#include "shared/config.h"
#include "pathplannermodule.h"

namespace PathPlanner{
using namespace Data;

PathPlannerHelper::PathPlannerHelper(PathPlannerModule *ppmodule, QObject *parent) :
    QObject(parent), ppmodule(ppmodule)
{    
    moveToThread(QApplication::instance()->thread());
}

PathPlannerHelper::~PathPlannerHelper()
{

}

void PathPlannerHelper::sendMessageToGUI()
{
    ppmodule->sendMessageToGui();
}

}
