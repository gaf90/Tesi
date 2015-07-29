#ifndef MESSAGEPRIORITYHANDLER_H
#define MESSAGEPRIORITYHANDLER_H

#include <QString>
#include <QTime>

namespace BaseStation{

class MessagePriorityHandler
{
public:
    MessagePriorityHandler(QString moduleOrType = "", double initRelevance = 0,
                           double initAffidability = 0, double initUserInterest = 0);

    /**
    * This function handles the adaptive autonomy due to a message archiviation/deletion
    */
    void archiviation(bool wasPositive);
    void archiviation();

    //! Computes the priority of a message of this type received now!
    double computePriority();

    //! Sets a new value to the relevance of the message type.
    void setRelevance(double newRelevance);

    void userInteraction();

    const QString getModuleName() const;

private:
    QString moduleOrTypeName;

    //! How much this kind of messages are relevant for the operator. Can be set manually outside from here!
    double relevance;

    //! How reliable is the module/type of messages. It depends on the archiviation result
    double reliability;

    //! How much the user takes into consideration this kind of messages, not neglecting them.
    double userInterest;

    //! How many messages arrive from this module (or of this type) in a time period
    int specificWorkload;

    QTime lastMessageTime;

    QTime lastInteractionTime;

    //deprecated
    int unhandledMessages;

    int nEquivalentUserInteractions;

    int reliabilityFactor;

    void updateWorkload();
};

}
#endif // MESSAGEPRIORITYHANDLER_H
