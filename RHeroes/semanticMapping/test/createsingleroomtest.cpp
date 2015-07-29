#include "createsingleroomtest.h"
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

namespace SemanticMapping{
    namespace Test{

CreateSingleRoomTest::CreateSingleRoomTest()
{
//    SLAM::Geometry::LineSegment l1 ( 1, 1, 3, 1) ;
//    SLAM::Geometry::LineSegment l2 ( 2, 1.2, 5, 0.9) ;
//    SLAM::Geometry::LineSegment l3 ( 3, 0.8, 4, 1) ;
//    SLAM::Geometry::LineSegment l4 ( 6, 1, 7, 1) ;
//    SLAM::Geometry::LineSegment l5 ( 7.3, 0.9, 9, 1) ;
//    SLAM::Geometry::LineSegment l6 ( 8, 1.2, 10, 1) ;
//    SLAM::Geometry::LineSegment l7 ( 15, 1, 17, 1) ;
//    SLAM::Geometry::LineSegment l8 ( 3, 5, 3, 7) ;
//    SLAM::Geometry::LineSegment l9 ( 3, 12, 3, 13) ;

    SLAM::Geometry::LineSegment l1 ( 1, 2, 1, 5) ;
    SLAM::Geometry::LineSegment l2 ( 0.9, 5, 3, 5.1) ;
    SLAM::Geometry::LineSegment l3 ( 3.1, 4.9, 3.1, 4) ;
    SLAM::Geometry::LineSegment l4 ( 3, 4, 5, 4) ;
    SLAM::Geometry::LineSegment l5 ( 5, 4, 5, 2) ;
    SLAM::Geometry::LineSegment l6 ( 5, 2, 3, 1) ;
    SLAM::Geometry::LineSegment l7 ( 3, 1, 1, 1.95) ;
    SLAM::Geometry::LineSegment l8 ( 6, 6, 6,2) ;
    SLAM::Geometry::LineSegment l9 ( 6, 2, 4, 0) ;
    maxX = 6;
    maxY = 6;
    minX = 1;
    minY = 0;
    qDebug() << "vediamo come funziona intersects di 3 e 6" <<endl;
    qDebug() << l3.intersects(l6) <<endl;

    bool isTrue = false;
    QList<CreateSingleRoomTest::initWallStruct> consecutive;
    QList<CreateSingleRoomTest::secondWallStruct> secondList;
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
    walls.append(list);
    qDebug() << "Inizia il test, e la lista ha questo n° di elementi" << list.size() << endl;
    for (int i = 0; i < list.size(); ++i)  {
         if (consecutive.size() == 0) {
            QList<SLAM::Geometry::LineSegment> prova;
            CreateSingleRoomTest::initWallStruct temp = { fabs(fmod(list[i].angle(), M_PI )), prova, false, false  };
            if ((fabs(fmod(temp.angle, M_PI)) >= M_PI/2 - ANGLE_THRESHOLD) &&
                    (fabs(fmod(temp.angle, M_PI)) <= M_PI/2 + ANGLE_THRESHOLD)) temp.vertical = true;
            if ((fabs(fmod(temp.angle, M_PI)) <= ANGLE_THRESHOLD) &&
                 (fabs(fmod(temp.angle, M_PI)) >= M_PI - ANGLE_THRESHOLD) )temp.orizontal = true;
            temp.line.append(list[i]);
            consecutive.append( temp );
        }
        else
        {
            isTrue = false;
            for (int j = 0; j < consecutive.size(); ++j)   {
                if (( fabs(fmod(list.at(i).angle(), M_PI )) >= consecutive[j].angle - WALL_ANGLE_TOLERANCE) &&
                        ( fabs(fmod(list.at(i).angle(), M_PI )) <= consecutive[j].angle + WALL_ANGLE_TOLERANCE))
                {
                    //Aggiorno l'angolo della struttura, TODO errore Qui, come gestico la struttura?
                    consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(list[i].angle(), M_PI ))) /
                            (consecutive[j].line.size() + 1);
                    //Aggiungo la linea alla struttura.
                    consecutive[j].line.append(list.at(i));
                    isTrue = true;
                }
            }
            //Nessuna parete trovata = nuova parete!
            if (!isTrue) {
                QList<SLAM::Geometry::LineSegment> prova;
                CreateSingleRoomTest::initWallStruct temp = { fabs(fmod(list[i].angle(), M_PI )), prova, false, false  };
                if ((fabs(fmod(temp.angle, M_PI)) >= M_PI/2 - ANGLE_THRESHOLD) &&
                        (fabs(fmod(temp.angle, M_PI)) <= M_PI/2 + ANGLE_THRESHOLD)) temp.vertical = true;
                if ((fabs(fmod(temp.angle, M_PI)) <= ANGLE_THRESHOLD) &&
                     (fabs(fmod(temp.angle, M_PI)) >= M_PI - ANGLE_THRESHOLD) )temp.orizontal = true;
                temp.line.append(list[i]);
                consecutive.append( temp );
            }
        }
    }
    for (int i = 0; i < consecutive.size(); i++ )
    {
        if ((fabs(fmod(consecutive[i].angle, M_PI)) >= M_PI/2 - ANGLE_THRESHOLD) &&
                (fabs(fmod(consecutive[i].angle, M_PI)) <= M_PI/2 + ANGLE_THRESHOLD)) consecutive[i].vertical = true;
        if ((fabs(fmod(consecutive[i].angle, M_PI)) <= ANGLE_THRESHOLD) ||
             (fabs(fmod(consecutive[i].angle, M_PI)) >= M_PI - ANGLE_THRESHOLD) ) consecutive [i].orizontal = true;
    }


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
           CreateSingleRoomTest::WallClusterHandler(secondList);

        }
        secondList.clear();
        for (int i = 0; i < consecutive[z].line.size(); i++)  {
            if ( secondList.size() == 0 ) {
                QList<SLAM::Geometry::LineSegment> altraProva;
                CreateSingleRoomTest::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                           consecutive.at(z).line.at(i).q(),
                                                          - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                          altraProva};
                temp2.line.append(consecutive[z].line[i]);
                secondList.append(temp2);
            }
            else
            {
                isTrue = false;
                //MODIFICATE LE SUCCESSIVE RIGHE
                if ((consecutive[z].orizontal) || (consecutive[z].vertical) )
                    {
                    secondList[0].line.append( consecutive[z].line[i]);
                    isTrue = true;
                    qDebug() << consecutive[z].orizontal << endl;
                    qDebug() << consecutive[z].vertical << endl;
                }
                else {
                    for (int j = 0; j < secondList.size(); ++j)   {
                        if ((consecutive[z].line.at(i).q() >= secondList[j].Q - M_TOLERANCE) &&
                                (consecutive[z].line.at(i).q() <= secondList[j].Q + M_TOLERANCE) &&
                                (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                                 (secondList[j].QM - QM_TOLERANCE) ) &&
                                (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                                 secondList[j].QM + QM_TOLERANCE))
                        {
                            secondList[j].M = (secondList[j].M * secondList[j].line.size() + consecutive[z].line.at(i).m()) /
                                    (secondList[j].line.size()+1);
                            secondList[j].Q = (secondList[j].Q * secondList[j].line.size() + consecutive[z].line.at(i).q()) /
                                    (secondList.at(j).line.size()+1);
                            secondList[j].QM = -secondList[j].Q / secondList[j].M;
                            secondList[j].line.append(consecutive[z].line.at(i));
                            isTrue = true;
                        }
                    }
                }
                if (!isTrue) {
                    QList<SLAM::Geometry::LineSegment> altraProva;
                    CreateSingleRoomTest::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                               consecutive.at(z).line.at(i).q(),
                                                              - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                              altraProva};
                    temp2.line.append(consecutive[z].line[i]);
                    secondList.append(temp2);
                }
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
    CreateSingleRoomTest::WallClusterHandler(secondList);

    //TestIniziaQUI, gestirlo come un main da qui in poi!
    CreateRooms();
}

void CreateSingleRoomTest::WallClusterHandler( QList<secondWallStruct> list )
{
    bool swapped = false;
    bool vertical = false;
    double distance = 0;
    for (int i = 0; i < list.size(); i++ )
    {
        vertical = false;
        if (list[i].line[0].angle() <= M_PI/4) {
            vertical = true;
            do{
                swapped = false;
                for (int j = 1; j < list[i].line.size(); j++ ) {
                    //TODO CAMBIARE IL CAZZO DI CENTROIDE
                    //MODIFICATO
                    if (list[i].line[j-1].vertexMaxX().x() > list[i].line[j].vertexMaxX().x()) {
                    list[i].line.swap( j, j-1 );
                    swapped = true;
                    }
                }

            }
                while(swapped);
        }
        else {do{
                swapped = false;
                for (int j = 1; j < list[i].line.size(); j++ ) {
                    //TODO CAMBIARE IL CAZZO DI CENTROIDE
                    if (list[i].line[j-1].vertexMaxY().y() > list[i].line[j].vertexMaxY().y()) {
                    list[i].line.swap( j, j-1 );
                    swapped = true;}

                }
            }
                while(swapped);}
        for (int j = 1; j < list[i].line.size(); j++) {
            if(vertical)
                distance = - list[i].line[j-1].vertexMaxX().x() + list[i].line[j].vertexMinX().x();
            else distance =  - list[i].line[j-1].vertexMaxY().y() + list[i].line[j].vertexMinY().y();
            qDebug() << "distance vale " << distance <<endl;
            if ((distance > CONSEC_THRESHOLD) && (distance < DOOR_THRESHOLD))
            {
                if(vertical)
                {
                    swapped = false;
                    SemanticMapping::Geometry::DoorLineSegment temporary(
                                list[i].line[j-1].vertexMaxX(), list[i].line[j].vertexMinX());
                    qDebug() << "sto provando ad aggiungere una porta." <<endl;
                    for (int k = j; k < list[i].line.size(); k++)
                        if (list[i].line[j-1].vertexMaxX().x() >= list[i].line[k].vertexMinX().x())
                            swapped = true;

                    if (!swapped) {doors.append(temporary);
                        const double C = list[i].line[j-1].vertexMaxX().x();
                        const double D = list[i].line[j].vertexMinX().x();
                        const double A = list[i].line[j-1].vertexMaxX().y();
                        const double B = list[i].line[j].vertexMinX().y();
                        qDebug() << "sto aggiungendo una porta con " << A <<
                                    "," << C <<  " e " <<  B  << "," << D <<endl;}
                }
                else
                {
                    swapped = false;
                    SemanticMapping::Geometry::DoorLineSegment temporary(
                                list[i].line[j-1].vertexMaxY(), list[i].line[j].vertexMinY());
                    qDebug() << "sto provando ad aggiungere una porta."  <<endl;
                    for (int k = j; k < list[i].line.size(); k++)
                        if (list[i].line[j-1].vertexMaxY().y() >= list[i].line[k].vertexMinY().y())
                            swapped = true;
                    if (!swapped) {doors.append(temporary);
                        const double C = list[i].line[j-1].vertexMaxX().x();
                        const double D = list[i].line[j].vertexMinX().x();
                        const double A = list[i].line[j-1].vertexMaxX().y();
                        const double B = list[i].line[j].vertexMinX().y();
                        qDebug() << "sto aggiungendo una porta con " << A <<
                                    "," << C <<  " e " <<  B  << "," << D <<endl;}
                }
            }
        }
}
}


void CreateSingleRoomTest::CreateRooms()
{
    QList<SLAM::Geometry::Point> initPoints;
    int numberOfPoints = (int) (( maxX - minX ) * (maxY - minY) / POINTS_PER_SQUARE_METER );
    for (int i=0; i < numberOfPoints; i++)
    {
        // random() creates always the same sequence of numbers. If different sequences are needed, this method should be changed
        initPoints.append( * new SLAM::Geometry::Point(
                                (double) rand() / (RAND_MAX + 1.0) * (maxX - minX) + minX ,
                                (double) rand() / (RAND_MAX + 1.0) * (maxY - minY) + minY  ));
    }
    for (int i=0; i < numberOfPoints; i++) CreateSingleRoom(initPoints[i]);
}

void CreateSingleRoomTest::CreateSingleRoom(SLAM::Geometry::Point p)
{   SemanticMapping::Room newRoom(p);
    bool intersects = false;
    const QList<SLAM::Geometry::LineSegment> list = walls;
    QList<SLAM::Geometry::LineSegment> newSegmentList;
    for ( int i=0; i < list.size(); i++)
    {
        newSegmentList.append ( * new SLAM::Geometry::LineSegment( p, list[i].centroid() ) );
    }
    qDebug() << "il punto di partenza è (" << p.x() << "," << p.y() << ") " <<endl;
    for ( int i=0; i< newSegmentList.size(); i++) {
        intersects = false;
        for (int j=0; j < list.size(); j++) {
            //MOD QUI
            if (newSegmentList[i].intersects(list[j]) && ( i != j ))
            {
                intersects = true;
                qDebug() << "interseco il segmento n° " << i << endl;
            }
        }
        for (int j=0; j < doors.size(); j++) {
            //MOD QUI
            if (newSegmentList[i].intersects(doors[i]) && (i != j) )
                intersects = true;
        }
        qDebug() << "il segmento di partenza è" << list[i].x1() << "," <<
                    list[i].y1() << " ed " << list[i].x2() << "," << list[i].y2() << endl;
        qDebug() << "il segmento in questione vale" << newSegmentList[i].x1() << "," <<
                    newSegmentList[i].y1() << " ed " << newSegmentList[i].x2() << "," << newSegmentList[i].y2() << endl;
        qDebug() << "ed intersects vale: " << intersects <<endl;
        if (!intersects) newRoom.addSegment( * new SLAM::Geometry::LineSegment(list[i]));
    }
    qDebug() << "sono fuori dal for della stanza e new room ha questo # di stanze: " << newRoom.getRoom().size() <<endl;
    qDebug() << "la stanza contiene:" <<endl;
    for ( int i=0; i < newRoom.getRoom().size(); i++ ) {
        qDebug() << newRoom.getRoom()[i].x1() << "," << newRoom.getRoom()[i].y1() <<
                    " ed " << newRoom.getRoom()[i].x2() << "," << newRoom.getRoom()[i].y2() <<endl;
    }
}

    }
}

