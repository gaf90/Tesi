#include "prmfunctiontovar.h"

namespace PRM{

using namespace SLAM;
using namespace SLAM::Geometry;
using namespace Data;

PRMFunctionTovar::PRMFunctionTovar(PRMAlgorithm* prm, PathPlannerPRM* planner, uint robotId): prmAlgorithm(prm), planner(planner), EvaluationFunction(robotId), sameFrontierCounter(0), previousFrontier(NULL)
{
    ldbg<<"Policy: Tovar"<<endl;
}

EvaluationRecords* PRMFunctionTovar::evaluateFrontiers(const QList<Frontier*> &f, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData){
    QElapsedTimer timerPath;
    timerPath.start();
    QList<Frontier*> frontiers=map.frontiers();
    frontierValue=new QHash<Frontier, QList<double> >();
    pathsList=new QHash<const Frontier, QList<PRMPath> >();
    bestPathList=new QHash<Frontier, PRMPath >();
    foreach(Frontier* f, frontiers){
        evaluateFrontier(f,map,batteryTime,powerSignalData);
    }
    //    foreach(Frontier* f, frontiers){
    //        ldbg<<endl<< "Tovar Function: frontier: "<<f->centroid()<<" values: ";
    //        QList<double> values= frontierValue->value(*f);
    //        ldbg<<values<<endl;
    //    }
    normalizeTovar();
    EvaluationRecords* eval = new EvaluationRecords();
    foreach(Frontier* f, frontiers){
        ldbg<< "Tovar Function: frontier: "<<f->centroid();
        ldbg<<" values normalized: ";
        QList<double> values= frontierValue->value(*f);
        ldbg<<values<<endl;

        int bestPath =0;
        double bestValue=0.0;

        for(int i=0; i<values.size();i++){
            double value=values.at(i);
            if(value>bestValue){
                bestValue=value;
                bestPath=i;
            }
        }

        ldbg<<"Best Path: "<<bestPath<<endl;
        ldbg<<"Best Value: "<<bestValue<<endl;
        eval->putEvaluation(*f,bestValue);
        if(pathsList->contains(*f)){
            QList<PRMPath> paths= pathsList->value(*f);
            PRMPath path= paths.at(bestPath);
            bestPathList->insert(*f,path);
        }
    }
    //        eval->normalize();
    double best=0.0;
    Frontier* bestFrontier=NULL;
    Frontier* secondBestFrontier = NULL;
    foreach(Frontier f1, *(eval->getEvaluatedFrontiers())){
        double v = eval->getEvaluation(f1);
        if(v>best){
            best=v;
            if(bestFrontier!=NULL){
                secondBestFrontier=new Frontier(*bestFrontier);
            }
            bestFrontier= new Frontier(f1);
        }
        //ldbg<<"Frontier: "<<f1.centroid()<<" value: "<<v<<endl;
    }
    if(bestFrontier!=NULL){
        if(bestPathList->contains(*bestFrontier)){
            if(previousFrontier!=NULL){
                if(bestFrontier->centroid().distance(previousFrontier->centroid())<=Config::PRM::movementRadius){
                    sameFrontierCounter++;
                    ldbg<<"Using the same frontier "<<sameFrontierCounter<<" times"<<endl;
                }
                else{
                    sameFrontierCounter=0;
                    previousFrontier=bestFrontier;
                }
            }
            else{
                sameFrontierCounter=0;
                previousFrontier=bestFrontier;
            }
            if(sameFrontierCounter>=5){
                if(secondBestFrontier!=NULL){
                    if(bestPathList->contains(*secondBestFrontier)){
                        ldbg<<"Using the second best frontier!!!!"<<endl;
                        ldbg<<endl<<"Best Frontier: "<<secondBestFrontier->centroid()<<endl;
                        ldbg<<"Best path: "<<bestPathList->value(*secondBestFrontier)<<endl<<endl;
                        prmAlgorithm->setCurrentPath(bestPathList->value(*secondBestFrontier));
                        planner->calculateActions(bestPathList->value(*secondBestFrontier),map);
                    }
                    else{
                        ldbg<<"Can't find a valid path"<<endl;
                        planner->noPathFound();
                    }
                }
                else{
                    ldbg<<"Can't find a valid frontier"<<endl;
                    planner->noPathFound();
                }
            }
            else{
                ldbg<<endl<<"Best Frontier: "<<bestFrontier->centroid()<<endl;
                ldbg<<"Best path: "<<bestPathList->value(*bestFrontier)<<endl<<endl;
                prmAlgorithm->setCurrentPath(bestPathList->value(*bestFrontier));
                planner->calculateActions(bestPathList->value(*bestFrontier),map);
            }
        }
        else{
            ldbg<<"Can't find a valid frontier"<<endl;
            planner->noPathFound();
        }
    }
    else{
        ldbg<<"Can't find a valid path"<<endl;
        planner->noPathFound();
    }
    ldbg << "PRM Function: path creation for "<<frontiers.size()<<" frontiers in: " << timerPath.elapsed()<<"ms"<<endl<<endl;

    return eval;
}

double PRMFunctionTovar::evaluateFrontier(const Frontier* frontier, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData){
    Point centroid= frontier->centroid();
    QList<PRMPath> paths= prmAlgorithm->getPaths(new Point(centroid.x(), centroid.y()));
    if(paths.empty()){
        QList<double> values= QList<double>();
        values.append(0);
        frontierValue->insert(*frontier,values);
    }
    else{
        const Pose* pose=map.lastRobotPose(Config::robotID);
        pathsList->insert(*frontier,paths);
        QList<double> values= QList<double>();
        foreach(PRMPath path, paths){
            double pathValue= 0;
            int m=path.length();
            for(int i=0;i<m;i++){
                Point* point=path.at(i);
                Frontier* bestFrontier= NULL;
                double distance= INFINITY;
                foreach(Frontier* f, map.frontiers()){
                    double d=point->distance(f->centroid());
                    if(d<distance){
                        distance=d;
                        bestFrontier=f;
                    }
                }
                double lvi=bestFrontier->length();
                double svi =0;
                if(i<m-1){
                    svi=path.at(i+1)->distance(bestFrontier->centroid());
                }
                double temp=exp(lvi-svi);

                //produttoria
                double prod=1;
                double theta= pose->getTheta();
                for(int j=0; j<i;j++){
                    Point* p1= path.at(j);
                    Point* p2= path.at(j+1);
                    Pose poseIni(p1->x(),p1->y(),theta);
                    Pose poseFin(p2->x(),p2->y(),0);
                    double oj=computeRotationFromPoses(poseIni,poseFin);
                    double sj=p1->distance(*p2);
                    double temp2= exp(-oj)/(sqrt(sj)+1);
                    theta=theta+oj;
                    prod=prod*temp2;
                }
                temp=temp*prod;
                pathValue=pathValue+temp;
                //ldbg<<"I: "<<i<<" prod: "<<prod<<" VALUE: "<<pathValue<<" lvi: "<<lvi<<" svi: "<<svi<<endl;
            }
            values.append(pathValue);
        }
        frontierValue->insert(*frontier,values);
    }

    return 0.0;
}

void PRMFunctionTovar::normalizeTovar(){
    double min= INFINITY;
    double max= -INFINITY;
    QList<Frontier> frontiers= frontierValue->keys();
    foreach(Frontier f, frontiers){
        QList<double> list= frontierValue->value(f);
        foreach(double value, list){
            if(value<min){
                min=value;
            }
            if(value>max){
                max=value;
            }
        }
    }
    foreach(Frontier f, frontiers){
        QList<double> list= frontierValue->value(f);
        QList<double> newList= QList<double>();
        foreach(double value, list){
            double newValue=(value-min)/(max-min);
            newList.append(newValue);
        }
        frontierValue->insert(f,newList);
    }
}

void PRMFunctionTovar::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command){
    //controllo del robot da parte dell'utente: manuale!
}

}

