#include "prmfunctionmcdm.h"

#include "criteriaPRM/batterycriterionprm.h"
#include "criteriaPRM/distancecriterionprm.h"
#include "criteriaPRM/informationgaincriterionprm_mcdm.h"
#include "criteriaPRM/transmissionprobabilitycriterionprm.h"
#include "criteriaPRM/criterianameprm.h"
#include "criteriaPRM/criterioncomparatorprm.h"

namespace PRM{

using namespace SLAM;
using namespace SLAM::Geometry;
using namespace Exploration;

PRMFunctionMCDM::PRMFunctionMCDM(PRMAlgorithm* prm, PathPlannerPRM* planner, uint robotId): prmAlgorithm(prm), planner(planner), EvaluationFunction(robotId),
    criteria(new QHash<QString,CriterionPRM*>()), activeCriteria(NULL), sameFrontierCounter(0), previousFrontier(NULL)
{
    PRMWeightReader reader;
    matrix=reader.parseFile();
    ldbg<<"Policy: MCDM"<<endl;
    foreach(QString s, matrix->getKnownCriteria()){
        QString name = s;
        double weight = matrix->getWeight(matrix->getNameEncoding(name));
        ldbg<<"Criterion "<<name<<" value: "<<weight<<endl;
        CriterionPRM *c = createCriterion(name, weight);
        if(c != NULL)
            criteria->insert(name, c);
    }
}

EvaluationRecords* PRMFunctionMCDM::evaluateFrontiers(const QList<Frontier*> &f, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData){
    QElapsedTimer timerPath;
    timerPath.start();
    QList<Frontier*> frontiers=map.frontiers();
    pathsList=new QHash<const Frontier, QList<PRMPath> >();
    bestPathList=new QHash<Frontier, PRMPath >();
    foreach(CriterionPRM* c, criteria->values()){
        c->clean();
    }
    if(activeCriteria==NULL){
        activeCriteria= new QList<CriterionPRM*>();
    }
    foreach(QString name, matrix->getActiveCriteria()){
        activeCriteria->append(criteria->value(name));
    }
    foreach(Frontier* f, frontiers){
        evaluateFrontier(f,map,batteryTime,powerSignalData);
    }
    //    foreach(Frontier* f, frontiers){
    //        ldbg<<endl<< "PRM Function: frontier: "<<f->centroid()<<" criteria values: "<<endl;
    //        foreach(CriterionPRM* c, *activeCriteria){
    //            QList<double> value=c->getEvaluation(f);
    //            ldbg <<c->getName()<<" : "<<value<<endl;
    //        }
    //    }
    foreach(CriterionPRM* c, *activeCriteria){
        c->normalize();
    }
    EvaluationRecords* eval = new EvaluationRecords();
    foreach(Frontier* f, frontiers){
        ldbg<< "MCDM Function: frontier: "<<f->centroid();
        ldbg<<", criteria values normalized: "<<endl;
        QList<QList<double> > frontierValues=QList<QList<double> >();
        foreach(CriterionPRM* c, *activeCriteria){
            QList<double> value=c->getEvaluation(f);
            ldbg << "   "<<c->getName()<<" : "<<value<<endl;
            frontierValues.append(value);
        }
        int bestPath =0;
        double bestValue=0.0;

        for(int j=0; j< Config::PRM::pathNumber;j++){
            //            ldbg<<"Path: "<<j<<" = 1";
            //            double temp=1.0;
            //            for(int i=0; i<frontierValues.size();i++){
            //                QString criteriaName=activeCriteria->at(i)->getName();
            //                double weight=matrix->getWeight(matrix->getNameEncoding(criteriaName));
            //                double value= frontierValues.at(i).at(j);
            //                temp=temp*value*weight;
            //                ldbg<<"*"<<value<<"*"<<weight;
            //            }
            //            ldbg<<" = "<<temp;
            //            ldbg<<endl;
            //            if(temp>bestValue){
            //                bestValue=temp;
            //                bestPath=j;
            //            }
            //            ldbg<<endl;
            //ldbg<<"Path: "<<j<<endl;
            qSort(activeCriteria->begin(), activeCriteria->end(), CriterionComparatorPRM(f,j));

//            ldbg<<"criteria values sorted: "<<endl;
            foreach(CriterionPRM* c, *activeCriteria){
                QList<double> value=c->getEvaluation(f);
//                ldbg <<c->getName()<<" : "<<value.at(j)<<endl;
                frontierValues.append(value);
            }
            CriterionPRM* lastCriterion= NULL;
            double final=0.0;
//            ldbg << "Criteria weight: "<< endl;
            for(int i=0; i<activeCriteria->length();i++){
                CriterionPRM* c=NULL;
                double weight=0.0;
                QList<QString> names;
                for(int k=i; k<activeCriteria->length();k++){
                    CriterionPRM* next= activeCriteria->at(k);
                    names.append(next->getName());
                }
                weight=matrix->getWeight(names);
//                ldbg<<names<<" = "<<weight<<endl;
                if(i==0){
                    c=activeCriteria->first();
                    QList<double> temp= c->getEvaluation(f);
                    final=final+temp.at(j)*weight;
                }
                else{
                    c=activeCriteria->at(i);
                    QList<double> temp= c->getEvaluation(f);
                    QList<double> tempLast= lastCriterion->getEvaluation(f);
                    double difference=temp.at(j)-tempLast.at(j);
                    final=final+difference*weight;
                }
                lastCriterion=c;
            }
//            ldbg<<"Value: "<<final<<endl;
            if(final>bestValue){
                bestValue=final;
                bestPath=j;
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
    //    eval->normalize();
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
        ldbg<<"Can't find a valid path. Best frontier NULL!"<<endl;
        planner->noPathFound();
    }
    ldbg << "PRM Function: path creation for "<<frontiers.size()<<" frontiers in: " << timerPath.elapsed()<<"ms"<<endl<<endl;
    delete activeCriteria;
    activeCriteria=NULL;
    return eval;
}

double PRMFunctionMCDM::evaluateFrontier(const Frontier* frontier, const Map &map, int batteryTime, QHash<uint,double> &powerSignalData){
    Point centroid= frontier->centroid();
    QList<PRMPath> paths= prmAlgorithm->getPaths(new Point(centroid.x(), centroid.y()));
    if(paths.empty()){
        foreach(CriterionPRM* c, *activeCriteria){
            c->setWorstValue(frontier);
        }
    }
    else{
        pathsList->insert(*frontier,paths);
        foreach(CriterionPRM* c, *activeCriteria){
//            ldbg<<"Evaluating criteria: "<< c->getName()<<endl;
            c->evaluate(frontier,paths,map,batteryTime,powerSignalData);
        }
    }

    return 0.0;
}

void PRMFunctionMCDM::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command){
    //controllo del robot da parte dell'utente: manuale!
}

CriterionPRM* PRMFunctionMCDM::createCriterion(QString name, double weight){
    CriterionPRM* toRet=NULL;
//    ldbg << "Creating criteria: " <<name<<endl;
    if(name == QString(BATTERY_PRM)){
        toRet = new BatteryCriterionPRM(weight, robotId);
    } else if (name == QString(INFORMATION_GAIN_PRM)) {
        toRet = new InformationGainCriterionPRM_MCDM(weight);
    } else if (name == QString(DISTANCE_PRM)){
        toRet = new DistanceCriterionPRM(weight, robotId);
    } else if (name == QString(TRANSMISSION_PROBABILITY_PRM)) {
        toRet = new TransmissionProbabilityCriterionPRM(weight, robotId);
    }
    return toRet;
}

}

