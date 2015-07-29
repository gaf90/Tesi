#include "victimmanager.h"
#include "baseStation/params.h"
#include "baseStation/graphicparams.h"
#include "shared/utilities.h"
#include <QPushButton>
#include <QHBoxLayout>
#include <QMessageBox>
#include <cmath>

using namespace Data;
namespace BaseStation{

VictimManager::VictimManager():
    victims(new QList<Victim*>()), gui(new QTableView()),
    datamodel(new QStandardItemModel(0,VICTIM_VIEWER_COLUMNS,0)),
    victimCount(0), selectedVictim(0),
    imagesViewer(new graphics::ShowVictimWidget())
{
    //headers of the table
    QString h = VICTIM_HEADERS;
    QStringList headers = h.split(",");
    int i = 0;
    for(; i < headers.size(); i++){
        datamodel->setHeaderData(i, Qt::Horizontal, headers.at(i));
    }
    datamodel->setHeaderData(i, Qt::Horizontal, VICTIM_DELETION_HEADER);
    gui->setModel(datamodel);
    gui->setEditTriggers(QAbstractItemView::NoEditTriggers);

    //connections for interaction with the operator
    connect(gui, SIGNAL(clicked(QModelIndex)), this, SLOT(onCellClicked(QModelIndex)));

    //final widget composition
    finalView = new QWidget();
    gui->setParent(finalView);
    imagesViewer->setParent(finalView);
    QHBoxLayout *l = new QHBoxLayout();
    l->addWidget(gui);
    l->addWidget(imagesViewer);
    finalView->setLayout(l);
}

void VictimManager::onVictimFound(const QList<QImage> &images, const Pose &p, uint discoveringRobot)
{
    Victim *v;
    v = new Victim(discoveringRobot, victimCount++, new Pose(p));
    for(int i = 0; i < images.size(); i++){
        v->addImage(images.at(i));
    }
    victims->append(v);
    createNewRow(v);
    emit signalVictimConfirmed(p,v->getVictimID(), v->getDiscoverererID(), true);
}

void VictimManager::onVictimAddedByUser(Pose position)
{
    Victim *v;
    v = new Victim(BASE_STATION_ID, victimCount++, new Pose(position));
    victims->append(v);
    createNewRow(v);
    emit signalVictimConfirmed(position, v->getVictimID(), BASE_STATION_ID, true);
}

void VictimManager::onVictimAssigned(uint victimID, uint robot,  int time)
{
    Victim *victim;
    bool found = false;
    for(int i = 0; i< victims->size() && !found; i++){
        victim = victims->at(i);
        if(victim->getVictimID() == victimID)
        {
            found = true;
        }
    }
    if(found)
    {
        victim->assignToRobot(robot);
        //Update visualization
        QModelIndex index = datamodel->indexFromItem(datamodel->findItems(QString::number(victimID)).at(0));
        datamodel->item(index.row(),VICTIM_ASSOCIATION_COLUMN)->setText(robotNameFromIndex(robot));
        //About time to reach the victim
        datamodel->item(index.row(),TIME_TO_REACH_VICTIM_COLUMN)->setText(QString::number(time));
    }
    else
        ldbg << "assigned robot " << robot << " to victim " << victimID << " failed: no such victim" << endl;
}

void VictimManager::onVictimSelected(uint id)
{
    Victim *victim;
    bool found = false;
    for(int i = 0; i< victims->size() && !found; i++){
        victim = victims->at(i);
        if(victim->getVictimID() == id)
        {
            found = true;
            selectedVictim = victim;
            imagesViewer->setImages(selectedVictim->getImages());
            QModelIndex index = datamodel->indexFromItem(datamodel->findItems(QString::number(id)).at(0));
            gui->selectRow(index.row());
        }
    }
}

void VictimManager::onVictimMoved(uint id, Pose position)
{
    Victim *victim;
    bool found = false;
    for(int i = 0; i< victims->size() && !found; i++){
        victim = victims->at(i);
        if(victim->getVictimID() == id)
        {
            found = true;
            victim->setPosition(position);
            QModelIndex index = datamodel->indexFromItem(datamodel->findItems(QString::number(id)).at(0));
            datamodel->removeRow(index.row());
            createNewRow(victim);
        }
    }
    //TODO: Send message to singl robots! (to all of them!)
}

void VictimManager::onVictimDeletionRequested(Victim *victim, int row)
{
    //visualizzo un dialog con tanto di foto, quindi confermo.
    int ret = QMessageBox::warning(gui, tr("Victim deletion"),
                                    "Are you sure you want to delete victim " +
                                       QString::number(victim->getVictimID()) + "?",
                                    QMessageBox::Yes | QMessageBox::No);
    if(ret == QMessageBox::Yes){
        datamodel->removeRow(row);
        Victim *v;
        bool end = false;
        for(int i = 0; i < victims->size() && !end; i++){
            v = victims->at(i);
            if(v->getVictimID() == victim->getVictimID()){
                victims->removeAt(i);
                end = true;
            }
        }
        if(selectedVictim != 0)
            if(selectedVictim->getVictimID() == victim->getVictimID())
                selectedVictim = 0;
        ldbg << "Victim " << victim->getVictimID() << "to be deleted..." << endl;
        emit signalVictimDeleted(victim->getVictimID(), victim->getAssignedRobotID());
    }
}

void VictimManager::onCellClicked(QModelIndex index)
{
    QString header = datamodel->headerData(index.column(), Qt::Horizontal).toString();
    if(header == VICTIM_DELETION_HEADER)
    {   //Deletion clicked cell TODO
        gui->selectRow(index.row());
        uint victimID = datamodel->item(index.row(),0)->text().toUInt();
        Victim *v;
        bool found = false;
        for(int i = 0; i < victims->size() && !found; i++)
        {
            v = victims->at(i);
            if(v->getVictimID() == victimID)
            {
                found = true;
            }
        }
        onVictimDeletionRequested(v,index.row());
    }
    else
    {
        gui->selectRow(index.row());
        uint victimID = datamodel->item(index.row(),0)->text().toUInt();
        Victim *v;
        bool found = false;
        for(int i = 0; i < victims->size() && !found; i++)
        {
            v = victims->at(i);
            if(v->getVictimID() == victimID)
            {
                found = true;
                selectedVictim = v;
            }
        }
        imagesViewer->setImages(selectedVictim->getImages());
        emit signalVictimSelected(selectedVictim->getVictimID());
    }
}

void VictimManager::createNewRow(Victim *victim)
{
    QList<QStandardItem*> newRow = QList<QStandardItem*>();
    QStandardItem* id = new QStandardItem(QString::number(victim->getVictimID()));
    newRow.append(id);
    QStandardItem* pose = new QStandardItem("(" + QString::number(victim->getPosition().getX()) + "," +
                                            QString::number(victim->getPosition().getY()) + ")");
    newRow.append(pose);
    QStandardItem* assigned = new QStandardItem(victim->printAssignedRobotName());
    newRow.append(assigned);
    QStandardItem* discoverer = new QStandardItem(robotNameFromIndex(victim->getDiscoverererID()));
    newRow.append(discoverer);
    QStandardItem* time = new QStandardItem(victim->printTimeToBeReached());
    newRow.append(time);

    //delete cell
    newRow.append(new QStandardItem(QIcon(":/koIconMessageManager"),""));
    datamodel->appendRow(newRow);
}

QWidget * VictimManager::getVictimsView()
{
    return finalView;
}

uint VictimManager::getVictimCount()
{
    return victimCount;
}

}
