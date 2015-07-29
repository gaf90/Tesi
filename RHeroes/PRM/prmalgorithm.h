#ifndef PRMALGORITHM_H
#define PRMALGORITHM_H

#include "shared/config.h"
#include <cmath>
#include "graph.h"
#include "testPRM/testprmalgorithm.h"
#include <QThread>
#include "aStarPRM/astaralgorithmprm.h"
#include "testPRM/testastarprm.h"

#define PLOT_TIMEOUT 30*1000

namespace PRM {

using namespace SLAM::Geometry;
using namespace SLAM;

class PRMAlgorithm : public QThread
{
    Q_OBJECT
    friend class TestPRM::TestPRMAlgorithm;
    friend class TestPRM::TestAStarPRM;
public:

    /**
      *Costruttore
    */
    PRMAlgorithm();

    PRMAlgorithm(QString folder);

    /**
      *Distruttore
    */
    ~PRMAlgorithm();

    /**
      *Creazione di un nuovo punto casuale nell'intorno del robot
      *@param newMap: mappa dell'ambiente
      *@return point: ritorna il punto random generato
    */
    Point* newRandomPoint(Map newMap);

    /**
      *Creazione di un nuovo punto casuale nell'intorno di una frontiera
      *@param frontier: frontiera di interesse
      *@param map: mappa di interesse
      *@return point: ritorna il punto random generato
    */
    Point* newRandomPointFrontier(Frontier* frontier, Map map, boolplus isAbove);

    /**
      *Controllo della validit� di un punto
      *@param point: punto da controllare
      *@param newMap: mappa dell'ambiente
      *@return ritorna true se il punto � in un'area valida che non � stata precedentemente campionata
    */
    bool visibilityCheck (Point* point, Map newMap);

    /**
      *Controllo della validit� di un punto appartenente ad una frontiera
      *@param point: punto da controllare
      *@param frontier: frontiera di interesse
      *@param newMap: mappa dell'ambiente
      *@return ritorna true se il punto � nell'intorno della frontiera in una zona valida della mappa
    */
    bool visibilityCheckFrontier (Point* point, Frontier* frontier, SLAM::Map newMap);

    /**
      *Controllo della validit� di un punto in una mappa non precedentemente campionata
      *@param point: punto da controllare
      *@param newMap: mappa dell'ambiente
      *@return ritorna true se il punto � in un'area valida che non � stata precedentemente campionata
    */
    bool visibilityCheckNewMap (Point* point, Map newMap);

    /**
      *Aggiornamento del grafo con campionatura della nuova area visibile
      *@param newMap: mappa dell'ambiente
    */
    void updatePRM(Map newMap);

    /**
      *Ricerca delle frontiere rimosse con l'ultimo aggiornamento della mappa
      *@param newMap: mappa dell'ambiente
      *@return lista delle frontiere rimosse
    */
    QList<Frontier> foundFrontiersRemoved(Map newMap);

    /**
      *Ricerca delle frontiere aggiunte con l'ultimo aggiornamento della mappa
      *@param newMap: mappa dell'ambiente
      *@return lista delle frontiere aggiunte
    */
    QList<Frontier> foundFrontiersAdded(Map newMap);

    void plot(Map map);

    QList<PRMPath> getPaths(Point* destination);

    void plotPaths(QList<PRMPath> paths, int frontierID);

    bool comapareDouble(double a, double b);

    void setCurrentPath(PRMPath path);

    Data::Pose getOldRobotPose();

signals:
    void sigStartTimerPP();
    void sigStopTimerPP();

public slots:
    void handleNewMap(Map map);

private slots:
    void onStartTimerPP();
    void onStopTimer();
    void onTimeout();

protected:
    virtual void run();

private:
    QString pathFolder;
    Graph graph;
    Map oldMap;
    int iterationNumber;
    AStarAlgorithmPRM* aStar;
    int frontierID;
    QMutex* stateMutex;
    PRMPath currentPath;
    QTimer* plotTimer;
    double xRobot, yRobot;
    QList<QString> color;
    QList<Frontier> frontiersAdded;
    QList<Frontier> frontiersRemoved;
    QString fullPath;
    QString dataPath;
    QTime startTime;
};
}

#endif // PRMALGORITHM_H
