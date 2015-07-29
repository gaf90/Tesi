#ifndef NOTIFICATIONMANAGER_H
#define NOTIFICATIONMANAGER_H

#include <QObject>
#include "graphics/modules/notificationtableview.h"
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSortFilterProxyModel>
#include "data/infomessage.h"
#include "data/pose.h"
#include "baseStation/utils/victim.h"
#include "graphics/utils/showvictimdialog.h"
#include "baseStation/utils/messagepriorityhandler.h"

namespace BaseStation{

/**
* This class handles the message notifications to be shown to the operator.
* It must be able to filter unreliable or not interesting information and to focus on important,
* critical messages.
* It indexes all received messages, by time, by priority, by sender, etc.
* It handles message archiviation on user command, and evenctually on robot newest message... or for
* time expiration.
*/
class NotificationManager : public QObject
{
    Q_OBJECT
public:
    explicit NotificationManager(QObject *parent = 0);

    //! This one returns a pointer to the visualization component of the notification module
    QTabWidget * getMessageManagerViewer();


signals:

    void signalVictimFound(const QList<QImage> &images, const Data::Pose &p, uint discoveringRobot);

    void signalShowPopup(int id, Data::Pose position, QString module, QString message);

    void signalFocusOnRobot(uint robotID);
    void signalFocusOnPose(const Data::Pose &pose);

    void signalRobotChanged(uint id);

public slots:


//----------------------------SLOTS that handle incoming data-------------------------------------//

    //TODO and Check! Valutiamo la possibilità di splittare la gestione dei messaggi in diversi slot
    //specifici...
    /** This slot handles a new incoming message. It must be properly classified to determine his
    *   priority and affidability. Even the content must be extracted and verified.
    */
    void onNewInfoMessageReceived(Data::InfoMessage mex, uint robotID);

    /*Messages to be properly handled!!! For everyone we must compute priority from affidability
      and other params, included the past user behaviour!
                            TODO
    */
    void onVictimDetected(uint robotID, const QImage &image, double confidence, const Data::Pose position);
    void onFaultDetected(QString message, uint robotID);
//    void onNewSemanticInfo(const QString &info, const uint robotID);

    void onRelevanceConfigurationChanged(int victimDetection, int semanticMapping,
                                         int feedbackMessage, int faultDetection);

    void onMinimalPriorityUpdated(int priority);


private slots:
    //----------------------------SLOT that handles the archiviation process--------------------------//

        void onUserInteraction(QModelIndex mexIndex);

        void onFilterOnActual(QModelIndex index);
        void onFilterOnHistorical(QModelIndex index);
        void onUnfilterActual();
        void onUnfilterHistorical();

        /**
        * This method handles the recognition of a victim by the user.
        */
        void onVictimRecognition(uint victimId);

        //! Slot that handles the refusing of a victim.
        void onVictimRefused(uint victimId);

private:
    //! The total number of messages that have been managed (it is used to handle messageID...)
    uint messageCount;

//----------------------MVC pattern used to manage incoming messages------------------------------//

    //It is possible to use also other king of views like popup messages in special situations
    //! Graphical components that show the messages in the UI.
    graphics::NotificationTableView *activeMessagesView;
    graphics::NotificationTableView *historicalMessagesView;
    QTabWidget * viewer;

    //! Models to be used to manage data!
    QStandardItemModel *activeMessagesModel;
    QStandardItemModel *historicalMessagesModel;

    //! Proxy models
    QSortFilterProxyModel *activeMessageProxy;
    QSortFilterProxyModel *historicalMessageProxy;


    //! Value of the params that are associated to a specific message type.
    QHash<QString, MessagePriorityHandler*> *typeParams;

    uint messageId;

    QHash<uint, Victim*> *possibleVictims;

    graphics::ShowVictimDialog *victimDialog;

    QHash<uint, uint> *lastFaultMessage;
    QHash<uint, uint> *lastFeedbackMessage;

    int minimalPriority;

    /**
    * Archives the message in the corresponding row. That message will not appear anymore on screen,
    * but can be retrieved in the hystorical archives.
    */
    void archiveMessage(int row);


    QString fromIndexToId(uint mexIndex);
    uint fromIdToIndex(QString id);

    int computePriority(QString type, int confidence, bool substituted = false);
};

}
#endif // NOTIFICATIONMANAGER_H
