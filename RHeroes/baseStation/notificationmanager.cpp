#include "notificationmanager.h"
#include "baseStation/params.h"
#include <QList>
#include "shared/utilities.h"

using namespace graphics;
namespace BaseStation{
NotificationManager::NotificationManager(QObject *parent) :
    QObject(parent), messageCount(0),
    activeMessagesView(new NotificationTableView()),
    historicalMessagesView(new NotificationTableView()),
    viewer(new QTabWidget()),
    activeMessagesModel(new QStandardItemModel(0,MESSAGE_VIEWER_COLUMS,0)),
    historicalMessagesModel(new QStandardItemModel(0,MESSAGE_VIEWER_COLUMS-2,0)),
    activeMessageProxy(new QSortFilterProxyModel()), historicalMessageProxy(new QSortFilterProxyModel()),
    typeParams(new QHash<QString,MessagePriorityHandler*>()), messageId(1),
    possibleVictims(new QHash<uint, Victim*>()), victimDialog(0), lastFaultMessage(new QHash<uint, uint>()),
    lastFeedbackMessage(new QHash<uint, uint>()), minimalPriority(0)
{
    //Potrei realizzare qualcosa di un po' più scalabile! I tipi di messaggi potrebbero essere
    //letti dai messaggi stessi e quindi determinati dal wss controller ad esempio! O magari
    //posso fare una combinazione delle due cose: inizializzo solo i messaggi più importanti
    //esplicitamente e gli altri li inizializzo quando arrivano per la prima volta!

    //Inizializzazione dei messageParams per i diversi tipi di messaggi previsti!
    QString m = POSSIBLE_MESSAGES;
    QStringList mexTypes = m.split(",");
    QStringList values;
    for(int i = 0; i< mexTypes.size(); i++){
        values = mexTypes.at(i).split("-");
        typeParams->insert(values.at(0),new MessagePriorityHandler(values.at(0), values.at(1).toDouble(),
                                                      values.at(2).toDouble(), values.at(3).toDouble()));
    }

//Setting up dell'architettura MVC, con le varie relazioni fra componenti!
    //Headers
    QString h = NOTIFICATION_HEADERS;
    QStringList headers = h.split("/");
    int i = 0;
    for(; i < headers.size(); i++){
        historicalMessagesModel->setHeaderData(i, Qt::Horizontal, headers.at(i));
        activeMessagesModel->setHeaderData(i, Qt::Horizontal, headers.at(i));
    }
    activeMessagesModel->setHeaderData(i, Qt::Horizontal, ARCHIVIATION_HEADER);
    activeMessagesModel->setHeaderData(i+1, Qt::Horizontal, DELETION_HEADER);

//Setup dei proxy model per filtraggie e ordinamenti
    activeMessagesView->setModel(activeMessageProxy);
    historicalMessagesView->setModel(historicalMessageProxy);
    activeMessageProxy->setDynamicSortFilter(true);
    historicalMessageProxy->setDynamicSortFilter(true);
    activeMessageProxy->setSourceModel(activeMessagesModel);
    historicalMessageProxy->setSourceModel(historicalMessagesModel);
    activeMessageProxy->setFilterCaseSensitivity(Qt::CaseInsensitive);
    historicalMessageProxy->setFilterCaseSensitivity(Qt::CaseInsensitive);

//Connessioni varie fra i segnali della ui e gli slot di qui!
    connect(activeMessagesView, SIGNAL(signalFilteringRequested(QModelIndex)),
            this, SLOT(onFilterOnActual(QModelIndex)));
    connect(historicalMessagesView, SIGNAL(signalFilteringRequested(QModelIndex)),
            this, SLOT(onFilterOnHistorical(QModelIndex)));
    connect(activeMessagesView, SIGNAL(signalUnfilteringRequested()),
            this, SLOT(onUnfilterActual()));
    connect(historicalMessagesView, SIGNAL(signalUnfilteringRequested()),
            this, SLOT(onUnfilterHistorical()));
    connect(activeMessagesView, SIGNAL(clicked(QModelIndex)), this, SLOT(onUserInteraction(QModelIndex)));

//Composizione del viewer finale
    viewer->addTab(activeMessagesView,QIcon(),"Active Messages");
    viewer->addTab(historicalMessagesView, QIcon(), "Archived messages");

//Gestione sorting
    activeMessagesView->setSortingEnabled(true);
    historicalMessagesView->setSortingEnabled(true);

//Editing handling
    activeMessagesView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    historicalMessagesView->setEditTriggers(QAbstractItemView::NoEditTriggers);

    //TEST
//    onFaultDetected("test 1", 0);
//    onFaultDetected("test 2", 0);
//    onFaultDetected("test 3", 0);
//    onVictimDetected(0, QImage(":/victim1"),80,Data::Pose(3,4,1));
//    onVictimDetected(1, QImage(":/victim2"),70,Data::Pose(1,2,2));
//    onVictimDetected(0, QImage(":/victim3"),90,Data::Pose(3.1,4.1,1.2));

    activeMessagesView->sortByColumn(PRIORITY_COLUMN,Qt::DescendingOrder);
}

void NotificationManager::onNewInfoMessageReceived(Data::InfoMessage mex, uint robotID)
{
    //Feedback case!
    if(mex.getInfoName() != MOVEMENT_END)
        return;
    uint lastMessage = lastFeedbackMessage->value(robotID, 0);
    bool found = false;
    if(lastMessage != 0)
    {
        uint messageID;
        for(int i = 0; i < activeMessagesModel->rowCount() && !found; i++)
        {
            messageID = fromIdToIndex(activeMessagesModel->item(i, 0)->text());
            if(messageID == lastMessage)
            {
                activeMessagesModel->removeRow(i);
                found = true;
            }
        }
    }
    int priority = computePriority(FEEDBACK_TYPE, NM_MAXIMAL_PRIORITY, found);
    if(priority < minimalPriority)
        return;
    lastFeedbackMessage->insert(robotID, messageId);
    QList<QStandardItem*> x = QList<QStandardItem*>();
    x.append(new QStandardItem(fromIndexToId(messageId++)));
    x.append(new QStandardItem(QString::number(robotID)));
    x.append(new QStandardItem(QString::number(priority)));
    x.append(new QStandardItem(FEEDBACK_TYPE));
    if(mex.getInfoName() == MOVEMENT_END)
        x.append(new QStandardItem(robotNameFromIndex(robotID) + " reached the waypoint"));
    else
        x.append(new QStandardItem(robotNameFromIndex(robotID) + ": " + mex.getInfoName() + ", " + mex.getInfo()));
    x.append(new QStandardItem(QIcon(":/okIconMessageManager"),""));
    x.append(new QStandardItem(QIcon(":/koIconMessageManager"),""));
    activeMessagesModel->appendRow(x);
    viewer->setCurrentIndex(0);
}

void NotificationManager::onUserInteraction(QModelIndex index)
{
    QModelIndex mexIndex = activeMessageProxy->mapToSource(index);
    QString header = activeMessagesModel->headerData(mexIndex.column(), Qt::Horizontal).toString();
    uint robot = activeMessagesModel->item(mexIndex.row(), NN_ROBOT_COLUMN)->text().toUInt();
    MessagePriorityHandler* ph = typeParams->value(
                activeMessagesModel->item(mexIndex.row(), MODULE_COLUMN)->text(), 0);
    if(ph != 0)
        ph->userInteraction();
    else
        ldbg << activeMessagesModel->item(mexIndex.row(), MODULE_COLUMN)->text() << "is the module of the message" << endl << endl;
    if(header == ARCHIVIATION_HEADER){
        if(ph != 0)
            ph->archiviation(true);
        archiveMessage(mexIndex.row());
    }
    else if(header == DELETION_HEADER){
        if(ph != 0)
            ph->archiviation(false);
        activeMessagesModel->removeRow(mexIndex.row());
    }
    else if(activeMessagesModel->item(mexIndex.row(), MODULE_COLUMN)->text() == VICTIM_TYPE){
        ldbg << "interaction with a victim message" << endl;
        uint victimID = fromIdToIndex(activeMessagesModel->item(mexIndex.row(),0)->text());
        Victim *victim = possibleVictims->value(victimID, 0);
        activeMessagesView->selectRow(index.row());
        emit signalFocusOnPose(victim->getPosition());
        if(victim != 0){
            if(victimDialog == 0){
                victimDialog = new ShowVictimDialog(0, victimID, victim->getImages());
                connect(victimDialog, SIGNAL(signalNoVictimRetrieved(uint)),
                        this, SLOT(onVictimRefused(uint)));
                connect(victimDialog, SIGNAL(signalVictimConfirmed(uint)),
                        this, SLOT(onVictimRecognition(uint)));
                victimDialog->move(0,0);
                victimDialog->show(); //TO CHECK THIS!!!
            }
            else if(victimDialog->getVictimID() == victimID)
            {
                victimDialog->move(0,0);
                victimDialog->show(); //TO CHECK THIS!!!
            }
            else{
                delete victimDialog;
                victimDialog = new ShowVictimDialog(0, victimID, victim->getImages());
                connect(victimDialog, SIGNAL(signalNoVictimRetrieved(uint)),
                        this, SLOT(onVictimRefused(uint)));
                connect(victimDialog, SIGNAL(signalVictimConfirmed(uint)),
                        this, SLOT(onVictimRecognition(uint)));
                victimDialog->move(0,0);
                victimDialog->show(); //TO CHECK THIS!!!
            }
        }
        else return;
    }
    else //No victim mesage. We must show something to the user!!!
    {
        activeMessagesView->selectRow(index.row());
        emit signalFocusOnRobot(robot);
    }
}

QTabWidget * NotificationManager::getMessageManagerViewer()
{
    return this->viewer;
}



void NotificationManager::onFilterOnActual(QModelIndex index)
{
    if(index.column() >= 0 && index.column() < activeMessagesModel->columnCount()){
        activeMessageProxy->setFilterKeyColumn(index.column());
        QStandardItem* item = activeMessagesModel->item(index.row(), index.column());
        QString mex = item->text();
        activeMessageProxy->setFilterFixedString(mex);
    }
}

void NotificationManager::onFilterOnHistorical(QModelIndex index)
{
    if(index.column() >= 0 && index.column() < historicalMessagesModel->columnCount()){
        historicalMessageProxy->setFilterKeyColumn(index.column());
        historicalMessageProxy->setFilterFixedString(
                    historicalMessagesModel->item(index.row(), index.column())->text());
    }
}

void NotificationManager::onUnfilterHistorical()
{
    historicalMessageProxy->setFilterRegExp("");
}

void NotificationManager::onUnfilterActual()
{
    activeMessageProxy->setFilterRegExp("");
}

//Per ogni messaggio dovrei prima computare la priorità, dunque comportarmi in modo adeguato!!!
void NotificationManager::onFaultDetected(QString message, uint robotID)
{
    uint lastMessage = lastFaultMessage->value(robotID, 0);
    bool found = false;
    if(lastMessage != 0)
    {
        uint messageID;
        for(int i = 0; i < activeMessagesModel->rowCount() && !found; i++)
        {
            messageID = fromIdToIndex(activeMessagesModel->item(i, 0)->text());
            if(messageID == lastMessage)
            {
                activeMessagesModel->removeRow(i);
                found = true;
            }
        }
    }
    int priority = computePriority(FAULT_TYPE, NM_MAXIMAL_PRIORITY, found);
    if(priority < minimalPriority)
        return;
    lastFaultMessage->insert(robotID, messageId);
    QList<QStandardItem*> x = QList<QStandardItem*>();
    x.append(new QStandardItem(fromIndexToId(messageId++)));
    x.append(new QStandardItem(QString::number(robotID)));
    x.append(new QStandardItem(QString::number(priority)));
    x.append(new QStandardItem(FAULT_TYPE));
    x.append(new QStandardItem(message));
    x.append(new QStandardItem(QIcon(":/okIconMessageManager"),""));
    x.append(new QStandardItem(QIcon(":/koIconMessageManager"),""));
    activeMessagesModel->appendRow(x);
    viewer->setCurrentIndex(0);
}

void NotificationManager::onRelevanceConfigurationChanged(int victimDetection, int semanticMapping,
                                                          int feedbackMessage, int faultDetection)
{
    MessagePriorityHandler *handler;

    handler = typeParams->value(VICTIM_TYPE,0);
    if(handler != 0)
        handler->setRelevance(victimDetection);
    handler = typeParams->value(SEMANTIC_TYPE,0);
    if(handler != 0)
        handler->setRelevance(semanticMapping);
    handler = typeParams->value(FEEDBACK_TYPE,0);
    if(handler != 0)
        handler->setRelevance(feedbackMessage);
    handler = typeParams->value(FAULT_TYPE,0);
    if(handler != 0)
        handler->setRelevance(faultDetection);
}

void NotificationManager::onMinimalPriorityUpdated(int priority)
{
    this->minimalPriority = priority;
}

void NotificationManager::onVictimDetected(uint robotID, const QImage &image,
                                           double confidence, const Data::Pose position)
{
    uint idOldMex = 0;
    Victim *victim = new Victim(robotID, messageId, new Data::Pose(position));
    victim->addImage(image);
    bool found = false;
    foreach(Victim *v, possibleVictims->values()){
        if(v->equals(*victim)){
            //Add info to the already seen "victim"
            v->addImage(image);
            found = true;
            idOldMex = v->getVictimID();
            if(v->isConfirmed())
                return;
            break;
        }
    }
    int priority = computePriority(VICTIM_TYPE, confidence*100, found);
    if(priority < minimalPriority)
    {
        return;
    }
    if(!found){
        possibleVictims->insert(messageId, victim);
        QList<QStandardItem*> x = QList<QStandardItem*>();
        x.append(new QStandardItem(fromIndexToId(messageId++)));
        x.append(new QStandardItem(QString::number(robotID)));
        x.append(new QStandardItem(QString::number(priority)));
        x.append(new QStandardItem(VICTIM_TYPE));
        x.append(new QStandardItem("Possible Victim Detected!"));
        x.append(new QStandardItem(QIcon(":/okIconMessageManager"),""));
        x.append(new QStandardItem(QIcon(":/koIconMessageManager"),""));
        activeMessagesModel->appendRow(x);
        viewer->setCurrentIndex(0);
    }
    else{
        //Incremento la priorità del messaggio relativo a tale vittima!
        QList<QStandardItem*> items = activeMessagesModel->findItems("Message " + QString::number(idOldMex));
        if(items.size() > 0)
        {
            QStandardItem * item = items.at(0);
            QStandardItem * x = activeMessagesModel->item(item->row(), PRIORITY_COLUMN);
            int oldPriority = x->text().toInt();
            if(oldPriority < priority)
                x->setText(QString::number(priority));
        }
        delete victim;
    }
}

void NotificationManager::archiveMessage(int row)
{
    if(activeMessagesModel->item(row, MODULE_COLUMN)->text() == VICTIM_TYPE)//check for a victim detection confirmed
    {
        ldbg << "victim found and confirmed!" << endl;
        Victim * victim = possibleVictims->value(fromIdToIndex(activeMessagesModel->item(row, 0)->text()));
        emit signalVictimFound(victim->getImages(),victim->getPosition(),victim->getDiscoverererID());
    }
    QStandardItem * item;
    QList<QStandardItem*> archived = QList<QStandardItem*>();
    for(int i = 0; i < activeMessagesModel->columnCount()-2; i++){
        item = activeMessagesModel->item(row,i);
        item = new QStandardItem(item->text());
        archived.append(item);
    }
    activeMessagesModel->removeRow(row);
    historicalMessagesModel->appendRow(archived);
}

void NotificationManager::onVictimRecognition(uint victimId)
{
    QStandardItem* item = activeMessagesModel->findItems(fromIndexToId(victimId)).at(0);
    archiveMessage(activeMessagesModel->indexFromItem(item).row());
    if(victimDialog != 0){
        delete victimDialog;
        victimDialog = 0;
    }
}

void NotificationManager::onVictimRefused(uint victimId)
{
    QStandardItem* item = activeMessagesModel->findItems(fromIndexToId(victimId)).at(0);
    activeMessagesModel->removeRow(activeMessagesModel->indexFromItem(item).row());
    if(victimDialog != 0){
        delete victimDialog;
        victimDialog = 0;
    }
    //Remove from possible victims so to discard all the old related images
    possibleVictims->remove(victimId);
}

uint NotificationManager::fromIdToIndex(QString id)
{
    return id.split(" ").at(1).toUInt();
}

QString NotificationManager::fromIndexToId(uint mexIndex)
{
    return "Message " + QString::number(mexIndex);
}

int NotificationManager::computePriority(QString type, int confidence, bool substituted)
{
    //TODO: implemento qualcosa di fico che evolva nel tempo!
    double priority;
    MessagePriorityHandler* x = typeParams->value(type,0);
    if(substituted)
    {
        x->archiviation();
    }
    if(x != 0)
    {
        priority = (x->computePriority() / NM_MAXIMAL_PRIORITY) * confidence;
        return priority;
    }
    else
    {
        ldbg << "There's something wrong!!! No message of type: " << type << endl;
        return confidence;
    }
}


}
