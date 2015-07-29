#include <QtCore/QCoreApplication>
#include "math.h"
#include "iostream"
#include "cmath"
#include <QPointF>
#include "ostream"
#include "semanticMapping/room.h"
#include "semanticMapping/semantichandler.h"
#include "slam/geometry/linesegment.h"
#include "slam/map.h"

//Classe di test delle pareti!
using namespace std;
typedef struct initWallStruct
{
    double angle;
    QList<SLAM::Geometry::LineSegment> line;
} initWallStruct;
typedef struct secondWallStruct
{
    double Q, M, QM;
    QList<SLAM::Geometry::LineSegment> line;
} secondWallStruct;

int main(int argc, char *argv[])
{
    //funzione createwalls;
    SLAM::Map map;
    SLAM::Geometry::LineSegment l1 = new SLAM::Geometry::LineSegment( 1, 1, 3, 1) ;
    SLAM::Geometry::LineSegment l2 = new SLAM::Geometry::LineSegment( 4, 1, 6, 1) ;
    SLAM::Geometry::LineSegment l3 = new SLAM::Geometry::LineSegment( 6, 1, 6, 3) ;
    SLAM::Geometry::LineSegment l4 = new SLAM::Geometry::LineSegment( 6, 3, 3, 4) ;
    SLAM::Geometry::LineSegment l5 = new SLAM::Geometry::LineSegment( 4, 4, 4, 6) ;
    SLAM::Geometry::LineSegment l6 = new SLAM::Geometry::LineSegment( 4, 6, 3, 6) ;
    SLAM::Geometry::LineSegment l7 = new SLAM::Geometry::LineSegment( 3, 6, 3, 4) ;
    SLAM::Geometry::LineSegment l8 = new SLAM::Geometry::LineSegment( 1, 2, 3, 4) ;
    SLAM::Geometry::LineSegment l9 = new SLAM::Geometry::LineSegment( 1, 2, 1, 1) ;
    bool isTrue = false;
    initWallStruct *temp;
    secondWallStruct *temp2;
    QList<initWallStruct> consecutive;
    QList<secondWallStruct> secondList;
    const QList<SLAM::Geometry::LineSegment> list;
    list.insert(l1);
    list.insert(l2);
    list.insert(l3);
    list.insert(l4);
    list.insert(l5);
    list.insert(l6);
    list.insert(l7);
    list.insert(l8);
    list.insert(l9);
    cout << "Inizia il test, e la lista ha questo n° di elementi" << list.size() << endl;
    for (int i = 0; i < list.size(); ++i)  {
          cout <<  "Se è vuota consecutive, appendo il primo." <<endl;
         if (consecutive.size() == 0) {
            //TODO Sarà corretta sta roba? c'è uguale anche 30 righe sotto
            temp = (SemanticHandler::initWallStruct *) malloc(sizeof( SemanticHandler::initWallStruct));
            temp->angle = list.at(i).angle();
            temp->line.append(list.at(i));
            consecutive.append(*temp);
        }
        else
        {
            isTrue = false;
            for (int j = 0; j < consecutive.size(); ++j)   {
                if ((list.at(i).angle() >= consecutive[j].angle - WALL_ANGLE_TOLERANCE) &&
                        (list.at(i).angle() <= consecutive[j].angle + WALL_ANGLE_TOLERANCE))
                {
                    //Aggiorno l'angolo della struttura, TODO errore Qui, come gestico la struttura?
                    consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + list[i].angle()) /
                            (consecutive[j].line.size() + 1);
                    //Aggiungo la linea alla struttura.
                    consecutive[j].line.append(list.at(i));
                    isTrue = true;
                }
            }
            //Nessuna parete trovata = nuova parete!
            if (!isTrue) {
                temp = (SemanticHandler::initWallStruct * ) malloc(sizeof( SemanticHandler::initWallStruct));
                temp->angle = list.at(i).angle();
                temp->line.append(list.at(i));
                consecutive.append(*temp);
            }
        }
    }
    //TODO Adesso ho le pareti divise in una serie di subset di linesegment con lo stesso angolo. devo fare clustering su tale cosa per
    //trovare le pareti. Un Cluster = una parete! trovare dove si fa clustering!
    /*
    Come faccio? posso provare a trasformare i punti di ogni singolo subset attraverso l'angolo. A questo punto controllo
    quantizzo il tutto e ottengo degli intervalli = faccio un "istogramma". Tutti quelli che sono nella stessa colonna
    dell'istogramma saranno la stessa parete. Come step successivo ci piazzo le porte.
    Metodo migliore! ogni segmento è associato ad una retta
                    y = M * x + Q ed intercetta l'asse X in ( - Q / M ; 0 ) e l'asse Y in ( Q ; 0 ).
    Divido in cluster tutti i segmenti che hanno i due punti di intersezione vicini!
    */
    cout << "Ora stampo le cose che stanno dentro la prima struttura, ovvero quelli con angolo uguale" <<endl;
    cout << " la grandezza di consecutive è " << consecutive.size() <<endl;
    for (int count = 0; count <= consecutive.size(); count++ )
    {
        cout << "Ora stampo consecutive["<< count << "]" <<endl;
        for(int ccc = 0; ccc <=consecutive[count].line.size(); ccc++)
            cout << "che contiene" << consecutive[count].line[ccc].x1()
                    << "," << consecutive[count].line[ccc].y1()
                    << " e " << consecutive[count].line[ccc].x2()
                    << "," << consecutive[count].line[ccc].y2() << endl;
    }
//CI PROVO!
    for (int z = 0; z< consecutive.size(); z++)  {
        if( z != 0)
        {
        //TODO Aggiungiporta!

        }
        secondList.clear();
        for (int i = 0; i < consecutive.at(z).line.size(); ++i)  {
            //Se è vuota consecutive, appendo il primo.
            if ( secondList.size() == 0 ) {
                //TODO Sarà corretta sta roba? c'è uguale anche 30 righe sopra
                temp2 = (secondWallStruct *) malloc(sizeof( secondWallStruct));
                temp2->M = consecutive.at(z).line.at(i).m();
                temp2->Q = consecutive.at(z).line.at(i).q();
                temp2->QM = - temp2->Q / temp2->M ;
                secondList.append(*temp2);
            }
            else
            {
                isTrue = false;
                for (int j = 0; j < secondList.size(); ++j)   {
                    if ((consecutive[z].line.at(i).m() >= secondList[j].M - M_TOLERANCE) &&
                            (consecutive[z].line.at(i).m() <= secondList[j].M + M_TOLERANCE) &&
                            (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                             (secondList[j].QM - QM_TOLERANCE) ) &&
                            (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                             secondList[j].QM <+ QM_TOLERANCE))
                    {
                        //Aggiorno l'M  e Q e QM della struttura, TODO errore Qui, come gestico la struttura?
                        secondList[j].M = (secondList[j].M * secondList[j].line.size() + consecutive[z].line.at(i).m()) /
                                (secondList[j].line.size()+1);
                        secondList[j].Q = (secondList[j].Q * secondList[j].line.size() + consecutive[z].line.at(i).q()) /
                                (secondList.at(j).line.size()+1);
                        secondList[j].QM = - secondList[j].Q / secondList[j].M ;
                        //Aggiungo la linea alla struttura.
                        secondList[j].line.append(consecutive[z].line.at(i));
                        isTrue = true;
                    }
                }
                //Nessuna parete trovata = nuova singola parete!
                if (!isTrue) {
                    temp2 = (secondWallStruct *) malloc(sizeof( secondWallStruct));
                    temp2->M = consecutive[z].line.at(i).m();
                    temp2->Q = consecutive[z].line.at(i).q();
                    temp2->QM = - temp2->Q / temp2->M ;
                    secondList.append(*temp2);
                }
            }
        }
    }


}
