#include "rrtnearclosepair.h"

namespace PathPlanner{
RRTNearClosePair::RRTNearClosePair(RRTNode * near, RRTNode * close) :
    near(near), close(close)
{
}

RRTNearClosePair::RRTNearClosePair() :
    near(NULL), close(NULL)
{

}

RRTNearClosePair::~RRTNearClosePair()
{

}

RRTNode * RRTNearClosePair::getNear()
{
    return near;
}

void RRTNearClosePair::setNear(RRTNode * node)
{
    this->near = node;
}

RRTNode * RRTNearClosePair::getClose()
{
    return close;
}

void RRTNearClosePair::setClose(RRTNode * node)
{
    this->close = node;
}
}
