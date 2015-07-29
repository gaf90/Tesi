#ifndef RRTNEARCLOSEPAIR_H
#define RRTNEARCLOSEPAIR_H

#include "rrtnode.h"

namespace PathPlanner {
/**
  * This class represents a pair of RRTNode where the close node
  * already belongs to the graph and the near node is a computed node.
  * The class does not have the ownership over these objects, so it does not
  * destroy them.
  */
class RRTNearClosePair
{
public:
    /**
      * Contructor of the class
      * @param near the near node (the computed one)
      * @param close the closest node (the one that already belongs to the graph)
      */
    RRTNearClosePair(RRTNode * near, RRTNode * close);
    /**
      * Contructor of the class
      */
    RRTNearClosePair();
    /**
      * Destructor of the class
      */
    virtual ~RRTNearClosePair();

    RRTNode * getNear();
    void setNear(RRTNode * node);
    RRTNode * getClose();
    void setClose(RRTNode * node);


private:
    RRTNode * near;
    RRTNode * close;
};
}

#endif // RRTNEARCLOSEPAIR_H
