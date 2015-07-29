#include "informationgaincriterionprm.h"
#include "criterianameprm.h"
#include "shared/config.h"

namespace PRM{
using namespace SLAM::Geometry;
using namespace SLAM;

InformationGainCriterionPRM::InformationGainCriterionPRM(double weight) :
    CriterionPRM(INFORMATION_GAIN_PRM, weight, true)

{
}

InformationGainCriterionPRM::~InformationGainCriterionPRM()
{

}

double InformationGainCriterionPRM::evaluate(const Frontier *frontier, QList<PRMPath> paths, const Map &map,
                                             int batteryTime, QHash<uint, double> &powerSignalData)
{
    Q_UNUSED(batteryTime) Q_UNUSED(powerSignalData)

            QList<double> length;

    foreach(PRMPath p, paths){
        QHash<Frontier,double> lengthList = QHash<Frontier,double>();
        foreach(Point* point, p){
            foreach(Frontier f, visibleFrontiers(*point,map)){

                double visibleLength = calculateVisibleLength(*point,f);
                if(lengthList.contains(f)){
                    if(visibleLength>lengthList.value(f)){
                        lengthList.insert(f,visibleLength);
                    }
                }
                else{
                    lengthList.insert(f,visibleLength);
                }
            }
        }
        //ldbg<<"Information Gain: "<<frontier->centroid()<<" path: "<<i<<endl;
        double value=0.0;
        foreach(Frontier f, lengthList.keys()){
            double visibleLength= lengthList.value(f);
            value=value+visibleLength;
            //ldbg<<"Frontier : "<<f.centroid()<<" distance: "<<distance<<" value: "<<f.length()/distance<<endl;
        }
        length.append(value);
    }
    insertEvaluation(frontier, length);
    return 0.0;
}


double InformationGainCriterionPRM::calculateVisibleLength(Point p, Frontier f){
    double radius = Config::PRM::maxPointDistance/3;
    ldbg<<"radius "<<radius<<endl;
    if((p.distance(f.p1())<=radius) && (p.distance(f.p2())<=radius)){
        return f.length();
    }
    else if((p.distance(f.p1())>radius) && (p.distance(f.p2())>radius)){
        return 0.0;
    }
    else{
        double r2=radius*radius;
        double xc= p.x();
        double yc=p.y();
        //line equation: y=mx+k
        double m = f.m();
        double q= f.q();
        //ldbg<<"M,K: "<<m<<" "<<k<<endl;
        double a=1+pow(m,2);
        double b=2*m*q-2*xc-2*m*yc;
        double c= pow(q,2)-2*yc*q-r2+pow(xc,2)+pow(yc,2);
        //ldbg<<"A,B,C: "<<a<<" "<<b<<" "<<c<<endl;
        double x1,x2;
        x1= ((-b+sqrt(pow(b,2)-4*a*c))/(2*a));
        x2= ((-b-sqrt(pow(b,2)-4*a*c))/(2*a));
        double y1= m*x1+q;
        double y2= m*x2+q;
        Point p1(x1,y1);
        Point p2(x2,y2);
        Point intersection;
        //ldbg<<"Calculate Visible Length: "<<p1<<p2<<endl;
        if(p1.distance(f.p1())<=f.length() && p1.distance(f.p2())<=f.length()){
            //p1 on the frontier
            intersection = p1;
        }
        else{
            intersection = p2;
        }
        //ldbg<<"Intersection point: "<<intersection<<endl;
        if(p.distance(f.p1())< radius){
            return intersection.distance(f.p1());
        }
        else{
            return intersection.distance(f.p2());
        }
    }
}

QList<Frontier> InformationGainCriterionPRM::visibleFrontiers(Point point,const Map &map){
    QList<Frontier> ret= QList<Frontier>();
    foreach(Frontier f, map.getFrontiers()){
        if(map.isReachable(point,f.centroid(),Config::PRM::movementRadius)!=False){
            ret.append(f);
        }
    }
    return ret;
}

}
