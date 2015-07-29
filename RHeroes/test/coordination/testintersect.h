#ifndef TESTINTERSECT_H
#define TESTINTERSECT_H

namespace Test{
class TestIntersect
{
public:
    TestIntersect();
    void doSpeedTest();
    void doPlannerSimulationTest();

private:
    double fRand(double fMin, double fMax);
};
}

#endif // TESTINTERSECT_H
