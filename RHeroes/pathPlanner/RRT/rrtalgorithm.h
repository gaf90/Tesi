#ifndef RRTALGORITHM_H
#define RRTALGORITHM_H

#include <QSet>
#include <QStack>
#include "rrtnode.h"
#include "rrtnearclosepair.h"
#include "slam/map.h"
#include "data/action.h"
#include "test/pathplanning/RRT/testrrtalgorithm.h"
#include "slam/geometry/linesegment.h"

namespace PathPlanner{

    /**
      * This class is an implementation of the RRT algorithm for path-planning
      */
    class RRTAlgorithm
    {
        friend class Test::TestRRTAlgorithm;
    public:
        /**
          * Constructor for th class
          * @param map the map in which the robot is situated
          * @param start the RRTNode that represents the actual pose of the robot. The algorithm does not own
          *     the object, so you must delete it manually.
          * @param goal the RRTNode that represents the destination of the robot. The algorithm does not own
          *     the object, so you must delete it manually.
          * @param constraint a value that indicates the maximum angle (in degree) that robot can
          *     perform in a rotation. If the value is equal to -1, there are no constraints.
          *
          */
        RRTAlgorithm(SLAM::Map *map, RRTNode *start, RRTNode *goal, double constraint=-1);

        /**
          * Destructor of the object
          */
        virtual ~RRTAlgorithm();

        /**
          * This method starts the algorithm.
          * @param delta the max movement (in meters) that the robot can perform in a traslation
          * @return a stack containing the sequences of action to perform in order to reach the goal.
          *     On top of the stack there is the first action to perform.
          */
        QStack<Data::Action *> * doAlgorithm(int delta);
    private:
        QStack<Data::Action *> *computeLowLevelAction();
        RRTNode * randomNode();
        RRTNode * nearNode(RRTNode *close, RRTNode *rand, int delta);
        RRTNode * closestNode(RRTNode *graph, RRTNode *rand);
        RRTNearClosePair * constrainedClosestNode(RRTNode *graph, RRTNode *rand, int delta);


        void unconstrainedAlgorithm(int delta);
        void constrainedAlgorithm(int delta);      

    private:
        SLAM::Map *map;
        RRTNode *start, *goal;
        double constraint;
        QSet<RRTNode *> *generated;

    };
}

#endif // RRTALGORITHM_H
