#ifndef VICTIMMANAGER_H
#define VICTIMMANAGER_H

#include <QObject>
#include <QTableView>
#include <QStandardItemModel>
#include <QLabel>
#include "baseStation/utils/victim.h"
#include "graphics/utils/showvictimwidget.h"

namespace BaseStation{

/**
* This clas manages the victims found and their association with robots for the final rescue.
*/
class VictimManager : public QObject
{
    Q_OBJECT
public:
    VictimManager();

    /**
    * @returns the graphical component containing information about victims
    */
    QWidget *getVictimsView();

    uint getVictimCount();

signals:

    /**
    * Signal emitted when a victim has been selected by the user. The whole UI must react
    * accordingly.
    * @param id the id of the selected victim.
    */
    void signalVictimSelected(uint id);

    /**
    * This signal is emitted wwhen a new victim has been found (and already confirmed by the operator)
    * @param position Data::Pose of the victim
    * @param victimID uint that identifies the victim
    */
    void signalVictimConfirmed(Data::Pose position, uint victimID, uint discoverer, bool isNew);

    /**
    * This signal is emitted when the operator deletes a victim. It will be sent to the assignedRobot
    * So that it can delete it once forever...
    * @param victimID the uint id of the victim to be removed.
    */
    void signalVictimDeleted(uint victimId, uint robotID);

public slots:

    /**
    * This slot handles the event "victim found". It must save the victim and show it in the proper
    * graphical component.
    * @param images a QList<QImage*> containing all the images of the victim captured from the first
    * encounter of the victim to the definitive detection and confirmation by the operator.
    * @param p Pose of the victim as received by the automatical system or the user.
    */
    void onVictimFound(const QList<QImage> &images, const Data::Pose &p, uint discoveringRobot);

    void onVictimAssigned(uint victimID, uint robot, int time);

    void onVictimSelected(uint id);

    void onVictimMoved(uint id, Data::Pose position);

    void onVictimAddedByUser(Data::Pose position);

private slots:

    /**
    * This slot handles the process of deletion of a victim. It must guarantee that the operator
    * has not deleted the victim by chance.
    */
    void onVictimDeletionRequested(Victim *victim, int row/*something*/);

//    /**
//    * This slot handles the scrolling of the images related to the selected victim.
//    */
//    void onNextImageScrolled();
//    void onPreviousImageScrolled();

    void onCellClicked(QModelIndex index);

private:

    QList<Victim*> *victims;

    //! Graphical component to show victims, structured as a table
    QTableView *gui;
    //! the model containing all the data concerning victims that must be show to the user.
    QStandardItemModel *datamodel;

    uint victimCount;
    Victim *selectedVictim;


    graphics::ShowVictimWidget *imagesViewer;

    QWidget *finalView;

    void createNewRow(Victim* victim);
};

}
#endif // VICTIMMANAGER_H
