#include "testsuite.h"
#include "test/pathplanning/RRT/testrrtnode.h"
#include "test/pathplanning/RRT/testrrtnodecomparator.h"
#include "shared/logger.h"
#include <QDebug>

namespace Test{
TestSuite::TestSuite(QObject *parent) :
    QObject(parent)
{
}
}

