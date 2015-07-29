#ifndef HYBRIDASTARACTION_H
#define HYBRIDASTARACTION_H


namespace PathPlanner{
class HybridAStarAction
{
public:
    HybridAStarAction();
    HybridAStarAction(double vr, double vl);
    double getVr();
    double getVl();
    bool operator==(HybridAStarAction &action);
    double getTimeEstimate();

private:
    double vr,vl;
};


inline bool HybridAStarAction::operator==(HybridAStarAction &action)
{
    return ((vr == action.getVr()) && (vl == action.getVl()));
}
}

#endif // HYBRIDASTARACTION_H
