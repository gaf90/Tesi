#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "criteria/criteriaName.h"
#include "criteria/batterycriterion.h"
#include "criteria/distancecriterion.h"
#include "criteria/informationgaincriterion.h"
#include "criteria/mcdmweightreader.h"
#include "criteria/criterioncomparator.h"
#include "criteria/userareacriterion.h"
#include "criteria/userdirectioncriterion.h"
#include "criteria/transmissionprobabilitycriterion.h"
#include "pathPlanner/pathplannerutils.h"



namespace Exploration {
using namespace SLAM;
using namespace Geometry;

    MCDMFunction::MCDMFunction(uint robotId) :
        EvaluationFunction(robotId), criteria(new QHash<QString, Criterion *>()),
        activeCriteria(NULL)
    {
        //read the weight from somewhere
        MCDMWeightReader weightReader;
        matrix = weightReader.parseFile();
        foreach(QString s, matrix->getKnownCriteria()){
            QString name = s;
            double weight = matrix->getWeight(matrix->getNameEncoding(name));
            ldbg<<"Criterion "<<name<<" value: "<<weight<<endl;
            Criterion *c = createCriterion(name, weight);
            if(c != NULL)
                criteria->insert(name, c);
        }

        //        QHash<QString, double> * configuration = weightReader.parseFile();
        //        foreach(QString crit, configuration->keys()){
        //            if(crit.contains("_")){
        //                jointCriteria.insert(crit, configuration->value(crit));
        //            } else {
        //                Criterion *criterion = createCriterion(crit, configuration->value(crit));
        //                if(criterion!=NULL){
        //                    criteria->append(criterion);
        //                }
        //            }
        //        }
        //        delete configuration;

    }

    Criterion * MCDMFunction::createCriterion(QString name, double weight)
    {
        Criterion *toRet = NULL;
        if(name == QString(BATTERY)){
            toRet = new BatteryCriterion(weight, robotId);
        } else if (name == QString(INFORMATION_GAIN)) {
            toRet = new InformationGainCriterion(weight);
        } else if (name == QString(DISTANCE)){
            toRet = new DistanceCriterion(weight, robotId);
        } else if (name == QString(USER_DIRECTION)){
            toRet = new UserDirectionCriterion(weight);
        } else if (name == QString(USER_AREA)) {
            toRet = new UserAreaCriterion(weight, robotId);
        } else if (name == QString(TRANSMISSION_PROBABILITY)) {
            toRet = new TransmissionProbabilityCriterion(weight, robotId);
        }
        return toRet;
    }

    MCDMFunction::~MCDMFunction()
    {
        delete matrix;
        foreach(QString k, criteria->keys()){
            delete criteria->value(k);
        }
        delete criteria;

    }

    double MCDMFunction::evaluateFrontier(
            const Frontier *frontier, const Map &map,
            int batteryTime, QHash<uint, double> &powerSignalData)
    {

        //Should keep the ordering of the criteria and the weight of each criteria combinations
        foreach(Criterion *c, *activeCriteria){
            c->evaluate(frontier, map, batteryTime, powerSignalData);
        }
        //for loop, over the criteria, to compute the utility of the frontier.

        return 0.0;
    }

     EvaluationRecords* MCDMFunction::evaluateFrontiers(const QList<SLAM::Geometry::Frontier *> &frontiers, const SLAM::Map &map,
                                                  int batteryTime, QHash<uint, double> &powerSignalData)
     {
         //Clean the last evaluation
         foreach(Criterion *c, criteria->values()){
             c->clean();
         }

         //Get the list of activeCriteria
         if(activeCriteria == NULL){
             activeCriteria = new QList<Criterion *>();
         }

         foreach(QString name, matrix->getActiveCriteria()){
             activeCriteria->append(criteria->value(name));
         }

         //Evaluate the frontiers
         foreach(Frontier *f, frontiers){
             //Check if the frontier is reachable.

             Point posePoint(f->centroid().x(), f->centroid().y());
             bool evaluate = PathPlanner::frontierFound(map.frontiers(), posePoint);
             double value = 0.0;
             if(evaluate){
                 //The frontier is reachable => evaluate them
                 value = evaluateFrontier(f, map, batteryTime, powerSignalData);
             } else {
                 //The frontier is not reachable => set the worst value
                 foreach(Criterion *c, *activeCriteria){
                     c->setWorstValue(f);
                 }
             }
         }
         //Normalize the values
         foreach(Criterion *c, *activeCriteria){
             c->normalize();
         }
         //Create the EvaluationRecords
         EvaluationRecords *toRet = new EvaluationRecords();
         foreach(Frontier *f, frontiers){
             qSort(activeCriteria->begin(), activeCriteria->end(), CriterionComparator(f));
             //apply the choquet integral

             Criterion *lastCrit = NULL;
             double finalValue = 0.0;
             for(int i=0; i<activeCriteria->length(); i++){
                 //ldbg << "MCDM : value of i "<<i<<endl;
                 //ldbg << "MCDM : size of active criteria "<<activeCriteria->length()<<endl;
                 Criterion *c = NULL;
                 double weight = 0.0;
                 //Get the list of criterion that are >= than the one considered
                 QList<QString> names;
                 for(int j=i ; j<activeCriteria->length(); j++){
                     Criterion *next = activeCriteria->at(j);
                     names.append(next->getName());
                 }

                 weight = matrix->getWeight(names);
                 //ldbg << names <<" with weight "<<weight<<endl;
                 if(i==0){
                     c = activeCriteria->first();
                     finalValue += c->getEvaluation(f) * weight;
                 } else {
                     c = activeCriteria->at(i);
                     double tmpValue = c->getEvaluation(f)-lastCrit->getEvaluation(f);                     
                     finalValue += tmpValue*weight;
                 }
                 lastCrit = c;
             }
             toRet->putEvaluation(*f, finalValue);

         }

         delete activeCriteria;
         activeCriteria = NULL;

         return toRet;
     }

     void MCDMFunction::onEnableUserCriteria(bool activate, const Data::HighLevelCommand *command)
     {
         matrix->changeCriteriaActivation(USER_DIRECTION, false);
         matrix->changeCriteriaActivation(USER_AREA, false);
         if(activate){
             bool expDir = command->exploreDirection();
             if(expDir){
                 matrix->changeCriteriaActivation(USER_DIRECTION, true);
                 UserDirectionCriterion *c = (UserDirectionCriterion *)criteria->value(USER_DIRECTION);
                 c->setAlphaX(command->getX());
                 c->setAlphaY(command->getY());
                 ldbg << "userDirection with weights: x = " << command->getX() <<", "<<command->getY()<<endl;
             } else {
                 matrix->changeCriteriaActivation(USER_AREA, true);
                 UserAreaCriterion *c = (UserAreaCriterion *)criteria->value(USER_AREA);
                 Point p(command->getX(), command->getY());
                 c->setPoint(p);
                 ldbg << "userArea with point: <" << command->getX() <<", "<<command->getY()<< ">" <<endl;
             }
         }
     }
}
