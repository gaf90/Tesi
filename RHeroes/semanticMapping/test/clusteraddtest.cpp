#include "clusteraddtest.h"
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
#include <QDebug>

//Test OK!

namespace SemanticMapping{
    namespace Test{

ClusterAddTest::ClusterAddTest()
{
    //funzione createwalls;
//    SLAM::Geometry::LineSegment l1 ( 1, 1, 3, 1) ;
//    qDebug() << "l'angolo di l1 è " << l1.angle() <<endl;
//    SLAM::Geometry::LineSegment l2 ( 4, 1, 6, 1) ;
//    SLAM::Geometry::LineSegment l3 ( 6, 1, 6, 3) ;
//    SLAM::Geometry::LineSegment l4 ( 6, 3, 3, 4) ;
//    SLAM::Geometry::LineSegment l5 ( 4, 4, 4, 6) ;
//    qDebug() << "l'angolo di l5 è " << fabs(fmod(l5.angle(),M_PI)) <<endl;
//    SLAM::Geometry::LineSegment l6 ( 4, 6, 3, 6) ;
//    SLAM::Geometry::LineSegment l7 ( 3, 6, 3, 4) ;
//    qDebug() << "l'angolo di l7 è " << fabs(fmod(l7.angle(),M_PI)) <<endl;
//    SLAM::Geometry::LineSegment l8 ( 1, 2, 3, 4) ;
//    qDebug() << "l'angolo di l8 è " << fabs(l8.angle()) <<endl;
//    SLAM::Geometry::LineSegment l9 ( 1, 2, 1, 1) ;
    SLAM::Geometry::LineSegment l1 ( 1, 1, 2, 2) ;
    SLAM::Geometry::LineSegment l2 ( 3, 3, 4, 4) ;
    SLAM::Geometry::LineSegment l3 ( 6, 6, 8, 8) ;
    SLAM::Geometry::LineSegment l4 ( 4, 1, 5, 2) ;
    SLAM::Geometry::LineSegment l5 ( 7, 0, 8, 1) ;
    SLAM::Geometry::LineSegment l6 ( 5, 5, 6, 4) ;
    SLAM::Geometry::LineSegment l7 ( 7, 3, 8, 1.95) ;
    SLAM::Geometry::LineSegment l8 ( 1, 4, 0, 5) ;
    SLAM::Geometry::LineSegment l9 ( 2, 5, 0, 7) ;
    bool isTrue = false;
    QList<ClusterAddTest::initWallStruct> consecutive;
    QList<ClusterAddTest::secondWallStruct> secondList;
    QList<SLAM::Geometry::LineSegment> list;
    list.append(l1);
    list.append(l2);
    list.append(l3);
    list.append(l4);
    list.append(l5);
    list.append(l6);
    list.append(l7);
    list.append(l8);
    list.append(l9);
    qDebug() << "Inizia il test, e la lista ha questo n° di elementi" << list.size() << endl;
    for (int i = 0; i < list.size(); ++i)  {
         qDebug() <<  "Se è vuota consecutive, appendo il primo." <<endl;
         if (consecutive.size() == 0) {
             qDebug() << "ciao, sono dentro il primo if, dovresti arrivare qui" <<endl;
            QList<SLAM::Geometry::LineSegment> prova;
            ClusterAddTest::initWallStruct temp = { fabs(fmod(list[i].angle() + M_PI , M_PI )), prova  };
            temp.line.append(list[i]);
            consecutive.append( temp );
            qDebug() << "ciao, sono alla fine del primo if, dovresti arrivare qui" << consecutive[0].line[0].length() <<endl;
        }
        else
        {
            isTrue = false;
            for (int j = 0; j < consecutive.size(); ++j)   {
                if (( fabs(fmod(list.at(i).angle() + M_PI, M_PI )) >= consecutive[j].angle - WALL_ANGLE_TOLERANCE) &&
                        ( fabs(fmod(list.at(i).angle() + M_PI, M_PI )) <= consecutive[j].angle + WALL_ANGLE_TOLERANCE))
                {
                    //Aggiorno l'angolo della struttura, TODO errore Qui, come gestico la struttura?
                    consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(list[i].angle() + M_PI, M_PI ))) /
                            (consecutive[j].line.size() + 1);
                    //Aggiungo la linea alla struttura.
                    consecutive[j].line.append(list.at(i));
                    isTrue = true;
                }
            }
            //Nessuna parete trovata = nuova parete!
            if (!isTrue) {
                QList<SLAM::Geometry::LineSegment> prova;
                ClusterAddTest::initWallStruct temp = { fabs(fmod(list[i].angle() + M_PI, M_PI )), prova  };
                temp.line.append(list[i]);
                consecutive.append( temp );
            }
        }
    }
    /*
    Come faccio? posso provare a trasformare i punti di ogni singolo subset attraverso l'angolo. A questo punto controllo
    quantizzo il tutto e ottengo degli intervalli = faccio un "istogramma". Tutti quelli che sono nella stessa colonna
    dell'istogramma saranno la stessa parete. Come step successivo ci piazzo le porte.
    Metodo migliore! ogni segmento è associato ad una retta
                    y = M * x + Q ed intercetta l'asse X in ( - Q / M ; 0 ) e l'asse Y in ( Q ; 0 ).
    Divido in cluster tutti i segmenti che hanno i due punti di intersezione vicini!
    */
    qDebug() << "Ora stampo le cose che stanno dentro la prima struttura, ovvero quelli con angolo uguale" <<endl;
    qDebug() << " la grandezza di consecutive è " << consecutive.size() <<endl;
    for (int count = 0; count < consecutive.size(); count++ )
    {
        qDebug() << "Ora stampo consecutive["<< count << "]" <<endl;
        qDebug() << "Che contiene "<<consecutive[count].line.size()<<endl;
        for(int ccc = 0; ccc <consecutive[count].line.size(); ccc++){
            qDebug() << "che contiene" << consecutive[count].line[ccc].x1()
                    << "," << consecutive[count].line[ccc].y1()
                    << " e " << consecutive[count].line[ccc].x2()
                    << "," << consecutive[count].line[ccc].y2() << endl;
        }
    }
//CI PROVO!
    for (int z = 0; z< consecutive.size(); z++)  {
        if( z != 0)
        {
            qDebug() << "Ora stampo le cose che stanno dentro la seconda struttura, ovvero quelli con M uguale" <<endl;
            qDebug() << " la grandezza di secondList è " << secondList.size() << "quando z="
                        << z << endl;
            for (int count = 0; count < secondList.size(); count++ )
            {
                qDebug() << "Ora stampo secondList["<< count << "]" <<endl;
                qDebug() << "Che contiene "<<secondList[count].line.size()<<endl;
                for(int ccc = 0; ccc <secondList[count].line.size(); ccc++){
                    qDebug() << "che contiene" << secondList[count].line[ccc].x1()
                            << "," << secondList[count].line[ccc].y1()
                            << " e " << secondList[count].line[ccc].x2()
                            << "," << secondList[count].line[ccc].y2() << endl;}}

        }
        secondList.clear();
         qDebug() << secondList.size() <<endl;
        for (int i = 0; i < consecutive[z].line.size(); ++i)  {
            //Se è vuota consecutive, appendo il primo.
            if ( secondList.size() == 0 ) {
                //TODO Sarà corretta sta roba? c'è uguale anche 30 righe sopra
                QList<SLAM::Geometry::LineSegment> altraProva;
                ClusterAddTest::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                           consecutive.at(z).line.at(i).q(),
                                                          - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                          altraProva};
                temp2.line.append(consecutive[z].line[i]);
                secondList.append(temp2);
            }
            else
            {
                isTrue = false;
                for (int j = 0; j < secondList.size(); ++j)   {
                    //TODO Modifica 24 maggio 2012;
                    qDebug() << "consecutive...q() vale " << consecutive[z].line.at(i).q() <<
                                " e invece secondlist.q vale " << secondList[j].Q <<endl;
                    qDebug() << "consecutive...QM() vale " <<consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m() <<
                                " e invece secondlist.q vale " <<  secondList[j].QM <<endl;
                    if ((consecutive[z].line.at(i).q() >= secondList[j].Q - M_TOLERANCE) &&
                            (consecutive[z].line.at(i).q() <= secondList[j].Q + M_TOLERANCE) &&
                            (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                             (secondList[j].QM - QM_TOLERANCE) ) &&
                            (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) <=
                             secondList[j].QM + QM_TOLERANCE))
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
                    QList<SLAM::Geometry::LineSegment> altraProva;
                    ClusterAddTest::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                               consecutive.at(z).line.at(i).q(),
                                                              - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                              altraProva};
                    temp2.line.append(consecutive[z].line[i]);
                    secondList.append(temp2);
                }
            }
        }

        qDebug() << "ed infine, l'ultimo cluster" <<endl;
        for (int count = 0; count < secondList.size(); count++ )
        {
            qDebug() << "Ora stampo secondList["<< count << "]" <<endl;
            qDebug() << "Che contiene "<<secondList[count].line.size()<<endl;
            for(int ccc = 0; ccc <secondList[count].line.size(); ccc++){
                qDebug() << "che contiene" << secondList[count].line[ccc].x1()
                        << "," << secondList[count].line[ccc].y1()
                        << " e " << secondList[count].line[ccc].x2()
                        << "," << secondList[count].line[ccc].y2() << endl;}}
    }


}
    }
}
