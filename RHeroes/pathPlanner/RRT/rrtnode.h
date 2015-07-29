#ifndef RRTNODE_H
#define RRTNODE_H

#include <QSet>
#include <QSetIterator>
#include <cmath>

namespace PathPlanner{
    /**
      * A class that represents a node for the RRT Algorithm
      */
    class RRTNode
    {
    public:
        /**
          * Constructor
          * @param x the x coordinate of the node
          * @param y the y coordinate of the node.
          */
        RRTNode(double x, double y);
        /**
          * Constructor
          */
        RRTNode();
        /**
          * Destructor
          */
        virtual ~RRTNode();

        /**
          * Getter for the x coordinate of the node
          * @return x coordinate
          */
        double getX() const;
        /**
          * Getter for the y coordinate of the node
          * @return y coordinate
          */
        double getY() const;
        /**
          * Getter for the orientation in which the robot will be directed when it
          * reache the node
          * return theta orientation
          */
        double getTheta() const;

        /**
          * Getter for the angle that the robot should perform to reach this node from the parent.
          * return the angle to rotate to reach this node.
          */
        double getSteeringAngle() const;

        /**
          * Getter for the parent node, the node from which the robot can reach this node
          * return the parent node.
          */
        RRTNode * getParent() const;
        /**
          * Getter for the validity of the node. A node is valid if:
          * - it is in the free space
          * - it is enough far from any obstacle
          * return <b>true</b> if the validity conditions are met; <b>false</b> otherwise.
          */
        bool isValid() const;
        /**
          * Getter for the iterator over the node that are directly reachable from this node.
          * If a node is directly reachable from this node, then this node is the parent for the reachable node
          * return an iterator.
          */
        QSetIterator<RRTNode *> * getReachableIterator() const;

        void setX(double x);
        void setY(double y);
        void setTheta(double theta);
        void setSteeringAngle(double steeringAngle);
        void setParent(RRTNode * parent);
        void setValid(bool valid);
        void addReachable(RRTNode * toAdd);

    private:
        double x, y, theta, steeringAngle;
        RRTNode * parent;
        QSet<RRTNode *> *reachable;

        bool valid;



    };

    inline bool operator==(const RRTNode &n1, const RRTNode &n2){
        if(n1.getX() != n2.getX())
            return false;
        if(n1.getY() != n2.getY())
            return false;
        if(n1.getTheta() != n2.getTheta())
            return false;
        if(n1.getSteeringAngle() != n2.getSteeringAngle())
            return false;
        if(n1.getParent() != n2.getParent())
            return false;
        if(n1.isValid() != n2.isValid())
            return false;
        return true;
    }

    inline bool operator!=(const RRTNode &n1, const RRTNode &n2){
        return !(n1==n2);
    }

    inline uint qHash(const RRTNode &n){
        uint prime = 31;
        uint result = 1;
        result = prime * result + n.getTheta();
        result = prime * result + n.getX();
        result = prime * result + n.getY();
        result = prime * result + n.getSteeringAngle();
        result = prime * result + qHash(*(n.getParent()));
        result = prime * result + n.isValid();
        return result;
    }

    inline double EuclideanDistance(const RRTNode &n1, const RRTNode &n2){
        return sqrt(pow((n1.getX()-n2.getX()), 2) + pow((n1.getY()-n2.getY()), 2));
    }
}

#endif // RRTNODE_H
