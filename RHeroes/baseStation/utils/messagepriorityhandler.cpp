#include "messagepriorityhandler.h"
#include "baseStation/params.h"
#include <math.h>

namespace BaseStation{

MessagePriorityHandler::MessagePriorityHandler(QString moduleOrType, double initRelevance,
                                               double initReliability, double initUserInterest):
    moduleOrTypeName(moduleOrType), relevance(initRelevance), reliability(initReliability),
    userInterest(initUserInterest), specificWorkload(0), unhandledMessages(0),
    nEquivalentUserInteractions(0), reliabilityFactor(50)
{
    lastMessageTime = QTime::currentTime();
    lastInteractionTime = QTime::currentTime();
}

const QString MessagePriorityHandler::getModuleName() const
{
    return this->moduleOrTypeName;
}

void MessagePriorityHandler::setRelevance(double newRelevance)
{
    this->relevance = newRelevance;
}

void MessagePriorityHandler::userInteraction()
{
    nEquivalentUserInteractions++;
    lastInteractionTime = QTime::currentTime();
}

void MessagePriorityHandler::updateWorkload()
{
    QTime time = QTime::currentTime();
    int timeFromLastMessage = lastMessageTime.secsTo(time);
    if( timeFromLastMessage > NM_TIME_THRESOLD)
        this->specificWorkload = 0;
    else{
        specificWorkload+= NM_TIME_THRESOLD - timeFromLastMessage;
    }

    // ------------------- User Interest Computation ----------------------- //
    nEquivalentUserInteractions -= lastInteractionTime.secsTo(time)/NM_TIME_FOR_INTERACTION_REDUCTION;
    if(nEquivalentUserInteractions < 1)
        nEquivalentUserInteractions = 1;
    else if(nEquivalentUserInteractions > NM_MAX_USER_INTEREST_FACTOR)
        nEquivalentUserInteractions = NM_MAX_USER_INTEREST_FACTOR;
    double interactionFactor = 1.0 / nEquivalentUserInteractions;
    double minInteractionFactor = 3.0;
    //gaussian distribution over the values of interactionFactor
    userInterest = (1./(minInteractionFactor * (sqrt(2*M_PI)/3))) *
            pow(M_E, -((9. * pow(interactionFactor,2)) / (2 * pow(minInteractionFactor,2))));

    // --------------------- Reliability Computation ----------------------- //
    double sigma = 3.;
    reliability = (1./(sigma * (sqrt(2*M_PI)/3))) *
            pow(M_E, -((pow(reliabilityFactor - NM_MAXIMAL_RELIABILITY, 2)) / (2 * pow(sigma,2))));

    lastMessageTime = time;
}

void MessagePriorityHandler::archiviation(bool wasPositive)
{
    if(wasPositive)
        this->reliabilityFactor += NM_SINGLE_MESSAGE_EFFECT;
    else
        this->reliabilityFactor -= NM_SINGLE_MESSAGE_EFFECT;
    unhandledMessages--;
}

void MessagePriorityHandler::archiviation()
{
    unhandledMessages--;
}

double MessagePriorityHandler::computePriority()
{
    unhandledMessages++;
    updateWorkload();
    double priority = (relevance * NM_RELEVANCE_WEIGHT + reliability * NM_AFFIDABILITY_WEIGHT +
                      userInterest * NM_USER_INTEREST_WEIGHT - specificWorkload * NM_WORKLOAD_WEIGHT) / NM_MAXIMAL_PRIORITY;
    return priority;
}

}
