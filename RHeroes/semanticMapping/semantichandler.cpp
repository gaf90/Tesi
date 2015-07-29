#include "semantichandler.h"
#include "SemanticDef.h"
#include "slam/map.h"
#include "semanticMapping/SemanticDef.h"
#include "semanticMapping/room.h"
//TODO Rimuovere
#include "shared/logger.h"

namespace SemanticMapping{

SemanticHandler::SemanticHandler(const SLAM::Map *inMap):
    map(inMap)
{
    CreateWalls();
    CreateRooms();
    RemoveRooms();
    ProcessRooms();
    MergeRooms();
    ldbg << "ho finito di fare il mio lavoro" <<endl;
}

//TODO
SemanticHandler::~SemanticHandler()
{
//    foreach(Room *r, rooms)
//        delete r;
//    foreach(Geometry::DoorLineSegment *d, doors)
//        delete d;
}

semanticInfos *SemanticHandler::getSemanticInfos(SLAM::Geometry::Point sample)
{
    semanticInfos *outValue;
    outValue = new semanticInfos();
    Room *stanza = SemanticHandler::getRoomPoint(sample);
    if (stanza == NULL) return NULL;
    outValue->label = stanza->getInfo();
    outValue->area = stanza->getArea();
    outValue->grade = stanza->getGrade();
    outValue->corridor_lenght = stanza->getCorridorLenght();
    return outValue;
}

//TODO Check se si pu passare una lista in un modo cos scemo.
QList<SemanticMapping::Room> SemanticHandler::getRooms()
{
    return rooms;
}

//TODO Modificare questa cosa che serve solo per debugging;
QList<SemanticMapping::Geometry::DoorLineSegment> SemanticHandler::getDoors()
{
    return doors;
}

Room* SemanticHandler::getRoomPoint(SLAM::Geometry::Point &p1)
{
    for (int i=0; i<rooms.size(); i++) {
        if (rooms[i].isInRoom(p1))
            return &rooms[i];
    }
    return NULL;
}

void SemanticHandler::CreateWalls()
{
    bool isTrue = false;
    QList<initWallStruct> consecutive;
    QList<secondWallStruct> secondList;
    const QList<SLAM::Geometry::LineSegment> list = map->getWalls();
    for (int i = 0; i < list.size(); ++i)  {
        //Se  vuota consecutive, appendo il primo.
        if ( list[i].length() > MIN_LENGHT_SEGMENT) {
        if (consecutive.size() == 0) {
            QList<SLAM::Geometry::LineSegment> prova;
            double angoloTemp = fabs(fmod (list[i].angle() + M_PI ,M_PI ));
            while (angoloTemp >= M_PI - 2*WALL_ANGLE_TOLERANCE)
                angoloTemp -= M_PI;
            SemanticHandler::initWallStruct temp = { angoloTemp , prova, false, false };
            temp.line.append(list[i]);
            consecutive.append( temp );
        }
        else
        {
            isTrue = false;
            for (int j = 0; j < consecutive.size(); ++j)   {
                if ((( fabs(fmod(list.at(i).angle() + M_PI, M_PI )) >= consecutive[j].angle - WALL_ANGLE_TOLERANCE) &&
                        ( fabs(fmod(list.at(i).angle() + M_PI, M_PI ))  <= consecutive[j].angle + WALL_ANGLE_TOLERANCE))||
                        (( fabs(fmod(list.at(i).angle() + M_PI, M_PI )) >= M_PI - 2*WALL_ANGLE_TOLERANCE) &&
                        (consecutive[j].angle < 2*WALL_ANGLE_TOLERANCE)))
                {
                    if (( fabs(fmod(list.at(i).angle() + M_PI, M_PI )) >= M_PI - 2*WALL_ANGLE_TOLERANCE) &&
                        (consecutive[j].angle < 2*WALL_ANGLE_TOLERANCE))
                            consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(list[i].angle() + M_PI, M_PI)) - M_PI) /
                                (consecutive[j].line.size() + 1);
                    else {
                    //Aggiorno l'angolo della struttura,
                    //TODO  Qui, come gestico la struttura? farlo in maniera pesata!
                    consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(list[i].angle() + M_PI, M_PI))) /
                            (consecutive[j].line.size() + 1);}
                    //Aggiungo la linea alla struttura.
                    consecutive[j].line.append(list.at(i));
                    isTrue = true;
                }
            }
            //Nessuna parete trovata = nuova parete!
            if (!isTrue) {
                QList<SLAM::Geometry::LineSegment> prova;
                double angoloTemp = fabs(fmod (list[i].angle() + M_PI ,M_PI ));
                while (angoloTemp >= M_PI - 2*WALL_ANGLE_TOLERANCE)
                    angoloTemp -= M_PI;
                SemanticHandler::initWallStruct temp = { angoloTemp , prova, false, false };
                temp.line.append(list[i]);
                consecutive.append( temp );
            }
        }
    }
    }
    for (int i = 0; i < consecutive.size(); i++ )
    {
        if ((fabs(fmod(consecutive[i].angle + M_PI, M_PI)) >= M_PI/2 - ANGLE_THRESHOLD) &&
                (fabs(fmod(consecutive[i].angle + M_PI , M_PI)) <= M_PI/2 + ANGLE_THRESHOLD)) consecutive[i].vertical = true;
        if ((fabs(fmod(consecutive[i].angle + M_PI, M_PI)) <= ANGLE_THRESHOLD) ||
             (fabs(fmod(consecutive[i].angle + M_PI, M_PI)) >= M_PI - ANGLE_THRESHOLD) ) consecutive [i].orizontal = true;
    }

    /*
    Come faccio? posso provare a trasformare i punti di ogni singolo subset attraverso l'angolo. A questo punto controllo
    quantizzo il tutto e ottengo degli intervalli = faccio un "istogramma". Tutti quelli che sono nella stessa colonna
    dell'istogramma saranno la stessa parete. Come step successivo ci piazzo le porte.
    Metodo migliore! ogni segmento  associato ad una retta
                    y = M * x + Q ed intercetta l'asse X in ( - Q / M ; 0 ) e l'asse Y in ( Q ; 0 ).
    Divido in cluster tutti i segmenti che hanno i due punti di intersezione vicini!
    */

    for (int z = 0; z< consecutive.size(); z++)  {
        if( z != 0)
        {
        SemanticHandler::WallClusterHandler(secondList);

        }
        secondList.clear();
        for (int i = 0; i < consecutive.at(z).line.size(); ++i)  {
            //Se  vuota consecutive, appendo il primo.
            if ( secondList.size() == 0 ) {
                QList<SLAM::Geometry::LineSegment> altraProva;
                SemanticHandler::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                           consecutive.at(z).line.at(i).q(),
                                                           - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                           altraProva,
                                                           consecutive[z].orizontal,
                                                           consecutive[z].vertical};
                temp2.line.append(consecutive[z].line[i]);
                secondList.append(temp2);
            }
            else
            {
                isTrue = false;
                if ( (consecutive[z].orizontal) || (consecutive[z].vertical) )
                    {
                    //TODO Modifica
                    for (int j=0; j < secondList.size(); j++) {
                        if ((secondList[j].orizontal)&&(consecutive[z].orizontal))
                        {
                            if ((consecutive[z].line.at(i).q() >= secondList[j].Q - M_TOLERANCE) &&
                                    (consecutive[z].line.at(i).q() <= secondList[j].Q + M_TOLERANCE))
                            {
                                secondList[j].line.append( consecutive[z].line[i]);
                                isTrue = true;
                                break;
                            }
                        }
                        if ((secondList[j].vertical)&&(consecutive[z].vertical))
                        {
                            if ((consecutive[z].line.at(i).x1() >= secondList[j].line[0].x1() - X_VERTICAL_TOLERANCE) &&
                                    (consecutive[z].line.at(i).x1() <= secondList[j].line[0].x1() + X_VERTICAL_TOLERANCE))
                            {
                                secondList[j].line.append( consecutive[z].line[i]);
                                isTrue = true;
                                break;
                            }
                        }
                    }
                }
                else {
                    for (int j = 0; j < secondList.size(); ++j)   {
                        if ((consecutive[z].line.at(i).q() >= secondList[j].Q - M_TOLERANCE) &&
                                (consecutive[z].line.at(i).q() <= secondList[j].Q + M_TOLERANCE) &&
                                (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) >=
                                 (secondList[j].QM - QM_TOLERANCE) ) &&
                                (0 -  (consecutive[z].line.at(i).q()/ consecutive[z].line.at(i).m()) <=
                                 secondList[j].QM + QM_TOLERANCE))
                        {
                            secondList[j].M = (secondList[j].M * secondList[j].line.size() + consecutive[z].line.at(i).m()) /
                                    (secondList[j].line.size()+1);
                            secondList[j].Q = (secondList[j].Q * secondList[j].line.size() + consecutive[z].line.at(i).q()) /
                                    (secondList.at(j).line.size()+1);
                            secondList[j].QM = -secondList[j].Q / secondList[j].M;
                            secondList[j].line.append(consecutive[z].line.at(i));
                            isTrue = true;
                            break;
                        }
                    }
                }
                //Nessuna parete trovata = nuova singola parete!
                if (!isTrue) {
                    QList<SLAM::Geometry::LineSegment> altraProva;
                    SemanticHandler::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
                                                               consecutive.at(z).line.at(i).q(),
                                                               - consecutive.at(z).line.at(i).q() / consecutive.at(z).line.at(i).m(),
                                                               altraProva,
                                                               consecutive[z].orizontal,
                                                               consecutive[z].vertical};
                    temp2.line.append(consecutive[z].line[i]);
                    secondList.append(temp2);
                }
            }
        }
    }
    SemanticHandler::WallClusterHandler(secondList);
    //Pulisco le porte di troppo
    //TODO DISATTIVATO PERCHE' LE LINEE NON VENGONO UNITE!
    bool removed = false;
    for (int i=doors.length() - 1; i > 0; i--)
    {
        double lunghezza = doors[i].length();
        removed = false;
        foreach (SLAM::Geometry::LineSegment d, list)
        {

            if ((!removed) && (d.intersects(doors[i])))
            {
                SLAM::Geometry::LineSegment mladenUnisciISegmenti(doors[i].centroid(), d.intersection(doors[i]));
                if (mladenUnisciISegmenti.length() <= lunghezza *0.25 )
                    removed = true;
            }
        }
        if(removed)
            doors.removeAt(i);
    }
}

void SemanticHandler::WallClusterHandler( QList<secondWallStruct> list )
{
    bool swapped = false;
    bool orizontal = false;
    double distance = 0;
    for (int i = 0; i < list.size(); i++ )
    {
        orizontal = false;
        if ((fabs(fmod(list[i].line[0].angle() + M_PI, M_PI))   <= M_PI/4) ||
                (fabs(fmod(list[i].line[0].angle() + M_PI, M_PI)) >= 3*M_PI/4))
        {
            orizontal = true;
            do{
                swapped = false;
                for (int j = 1; j < list[i].line.size(); j++ ) {
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
                    if (list[i].line[j-1].vertexMaxY().y() > list[i].line[j].vertexMaxY().y()) {
                    list[i].line.swap( j, j-1 );
                    swapped = true;}

                }
            }
                while(swapped);}
        for (int j = 1; j < list[i].line.size(); j++) {
            if(orizontal)
                distance = list[i].line[j].vertexMinX().x() - list[i].line[j-1].vertexMaxX().x();
            else distance = list[i].line[j].vertexMinY().y() - list[i].line[j-1].vertexMaxY().y() ;
            if ((distance > CONSEC_THRESHOLD) && (distance < DOOR_THRESHOLD))
            {
                if(orizontal)
                {
                    swapped = false;
                    SemanticMapping::Geometry::DoorLineSegment temporary(
                                list[i].line[j-1].vertexMaxX(), list[i].line[j].vertexMinX());
                    for (int k = j; k < list[i].line.size(); k++)
                        if (list[i].line[j-1].vertexMaxX().x() >= list[i].line[k].vertexMinX().x())
                            swapped = true;

                    if (!swapped) doors.append(temporary);
                }
                else
                {
                    swapped = false;
                    SemanticMapping::Geometry::DoorLineSegment temporary(
                                list[i].line[j-1].vertexMaxY(), list[i].line[j].vertexMinY());
                    for (int k = j; k < list[i].line.size(); k++)
                        if (list[i].line[j-1].vertexMaxY().y() >= list[i].line[k].vertexMinY().y())
                            swapped = true;
                    if (!swapped) doors.append(temporary);
                }
            }
        }
    //TODO Aggiungere porta che sta alla fine delle cose;
    if (list[i].orizontal){
        SLAM::Geometry::LineSegment inizio = list[i].line.first();
        SLAM::Geometry::LineSegment fine = list[i].line.last();
        SLAM::Geometry::LineSegment inizioCorto, inizioLungo, fineCorto, fineLungo;
        //Solve[(y2 - y1) == m (x2 - x1) && (x1 - x2)^2 + (y1 - y2)^2 == d^2, {x2, y2}]
        double m, m_2, d_2, x1, x2, y1, y2;
        bool inCo, inLu, fiCo, fiLu;
        m = inizio.m();
        m_2 = pow(m,2);
        x1 = inizio.vertexMinX().x();
        y1 = inizio.vertexMinX().y();
        d_2 = CONSEC_THRESHOLD * CONSEC_THRESHOLD;
        x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
        y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
        inizioCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        d_2 = DOOR_THRESHOLD*DOOR_THRESHOLD;
        x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
        y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
        inizioLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );

        m = fine.m();
        m_2 = pow(m,2);
        x1 = fine.vertexMaxX().x();
        y1 = fine.vertexMaxX().y();
        d_2 = CONSEC_THRESHOLD * CONSEC_THRESHOLD;
        x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
        y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
        fineCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        d_2 = DOOR_THRESHOLD*DOOR_THRESHOLD;
        x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
        y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
        fineLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );

        inCo = false; inLu = false; fiCo = false; fiLu = false;
        foreach(SLAM::Geometry::LineSegment l, map->getWalls())
        {
            if (!(l == inizio) && !( l == fine))
            {
                if (l.intersects(inizioCorto))
                    inCo = true;
                if (l.intersects(inizioLungo))
                    inLu = true;
                if (l.intersects(fineCorto))
                    fiCo = true;
                if (l.intersects(fineLungo))
                    fiLu = true;
            }
        }
        d_2 = pow((CONSEC_THRESHOLD + DOOR_THRESHOLD)/2 ,2 );
        if (!inCo && inLu)
        {
            m = inizio.m();
            m_2 = pow(m,2);
            x1 = inizio.vertexMinX().x();
            y1 = inizio.vertexMinX().y();
            x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
        }
        if (!fiCo && fiLu)
        {
        //Aggiungere  la porta dopo;
            m = fine.m();
            m_2 = pow(m,2);
            x1 = fine.vertexMaxX().x();
            y1 = fine.vertexMaxX().y();
            x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
        }
        }
    else
    {
        SLAM::Geometry::LineSegment inizio = list[i].line.first();
        SLAM::Geometry::LineSegment fine = list[i].line.last();
        SLAM::Geometry::LineSegment inizioCorto, inizioLungo, fineCorto, fineLungo;
        //Solve[(y2 - y1) == m (x2 - x1) && (x1 - x2)^2 + (y1 - y2)^2 == d^2, {x2, y2}]
        double m, m_2, d_2, x1, x2, y1, y2;
        bool inCo, inLu, fiCo, fiLu;
        m = inizio.m();
        if ( m <= 100 )
        {
            m_2 = pow(m,2);
            x1 = inizio.vertexMinY().x();
            y1 = inizio.vertexMinY().y();
            d_2 = CONSEC_THRESHOLD * CONSEC_THRESHOLD;
            x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            inizioCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
            d_2 = DOOR_THRESHOLD*DOOR_THRESHOLD;
            x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            inizioLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        }
        else
        {
            x1 = inizio.vertexMinY().x();
            y1 = inizio.vertexMinY().y();
            x2 = x1;
            y2 = y1 - CONSEC_THRESHOLD;
            inizioCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
            y2 = y1 - DOOR_THRESHOLD;
            inizioLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        }

        m = fine.m();
        if (m <= 100 )
        {
            m_2 = pow(m,2);
            x1 = fine.vertexMaxY().x();
            y1 = fine.vertexMaxY().y();
            d_2 = CONSEC_THRESHOLD * CONSEC_THRESHOLD;
            x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            fineCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
            d_2 = DOOR_THRESHOLD*DOOR_THRESHOLD;
            x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
            y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
            fineLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        }
        else
        {
            x1 = fine.vertexMaxY().x();
            y1 = fine.vertexMaxY().y();
            x2 = x1;
            y2 = y1 + CONSEC_THRESHOLD;
            fineCorto = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
            y2 = y1 + DOOR_THRESHOLD;
            fineLungo = SLAM::Geometry::LineSegment( x1, y1, x2, y2 );
        }

        inCo = false; inLu = false; fiCo = false; fiLu = false;
        foreach(SLAM::Geometry::LineSegment l, map->getWalls())
        {
            if (!(l == inizio) && !( l == fine))
            {
                if (l.intersects(inizioCorto))
                    inCo = true;
                if (l.intersects(inizioLungo))
                    inLu = true;
                if (l.intersects(fineCorto))
                    fiCo = true;
                if (l.intersects(fineLungo))
                    fiLu = true;
            }
        }
        d_2 = pow((CONSEC_THRESHOLD + DOOR_THRESHOLD)/2 ,2 );
        if (!inCo && inLu)
        {
            m = inizio.m();
            if ( m < 100) {
                m_2 = pow(m,2);
                x1 = inizio.vertexMinY().x();
                y1 = inizio.vertexMinY().y();
                x2 = (-sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
                y2 = -m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
                doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
            }
            else
            {
                x1 = inizio.vertexMinX().x();
                y1 = inizio.vertexMinX().y();
                x2 = x1;
                y2 = y1 - (CONSEC_THRESHOLD + DOOR_THRESHOLD)/2;
                doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
            }
        }
        if (!fiCo && fiLu)
        {
        //Aggiungere porta dopo
            m = fine.m();
            if ( m < 100 )
            {
                m_2 = pow(m,2);
                x1 = fine.vertexMaxY().x();
                y1 = fine.vertexMaxY().y();
                x2 = (sqrt( d_2 + d_2 * m_2 ) + x1 + m_2 * x1 ) / ( 1 + m_2 );
                y2 = m* sqrt(d_2*(1+m_2)) / (1 + m_2) - m * x1 + m * x1 / (1 + m_2) + pow(m,3) * x1 / (1 + m_2) + y1 ;
                doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
            }
            else
            {
                x1 = fine.vertexMaxX().x();
                y1 = fine.vertexMaxX().y();
                x2 = x1;
                y2 = y1 + (CONSEC_THRESHOLD + DOOR_THRESHOLD)/2;
                doors.append(Geometry::DoorLineSegment(x1,y1,x2,y2));
            }
        }
    }
    }

    //TODO Aggiungere porta che sta alla fine delle cose;
}

void SemanticHandler::CreateRooms()
{
    QList<SLAM::Geometry::Point> initPoints;
    double maxX, maxY, minX, minY;
    maxX = map->getMaxX();
    maxY = map->getMaxY();
    minX = map->getMinX();
    minY = map->getMinY();
    int numberOfPoints = (int) (( maxX - minX ) * (maxY - minY) / POINTS_PER_SQUARE_METER );
    for (int i=0; i < numberOfPoints; i++)
    {
        // random() creates always the same sequence of numbers. If different sequences are needed, this method should be changed
        initPoints.append( SLAM::Geometry::Point(
                                (double) rand() / (RAND_MAX + 1.0) * (maxX - minX) + minX ,
                                (double) rand() / (RAND_MAX + 1.0) * (maxY - minY) + minY  ));
    }
    for (int i=0; i < numberOfPoints; i++) CreateSingleRoom(initPoints[i]);

}

void SemanticHandler::CreateSingleRoom(SLAM::Geometry::Point p)
{   SemanticMapping::Room newRoom(p);
    bool intersects = false;
    QList<SLAM::Geometry::LineSegment> list = map->getWalls();
    QList<SLAM::Geometry::Frontier> frontaliere = map->getFrontiers();
    QList<SLAM::Geometry::LineSegment> newSegmentList;

    //Todo questo prima o poi andr rimosso, o no?
    for (int i=list.size()-1; i > 0; i--)
        if (list[i].length() < MIN_LENGHT_SEGMENT)
            list.removeAt(i);

    for ( int i=0; i < list.size(); i++)
    {
        newSegmentList.append (SLAM::Geometry::LineSegment( p, list[i].centroid() ) );
    }
    for ( int i=0; i< newSegmentList.size(); i++) {
        intersects = false;
        for (int j=0; j < list.size(); j++)  {
            if ((newSegmentList[i].intersects(list[j])) && ( i != j ))
                intersects = true;
        }
        for (int j=0; j < doors.size(); j++)  {
            if (newSegmentList[i].intersects(doors[j]))
                intersects = true;
        }
        for (int j=0; j < frontaliere.size(); j++) {
            if (newSegmentList[i].intersects(frontaliere[j]))
                intersects = true;
        }
        //TODO Check assunzione che newsegmentlist.size = list.size e che siano ordinati assieme.
        if (!intersects)
            newRoom.addSegment( SLAM::Geometry::LineSegment(list[i]));
    }


    newSegmentList.clear();

    for ( int i=0; i < doors.size(); i++)
    {
        newSegmentList.append ( SLAM::Geometry::LineSegment( p, doors[i].centroid() ) );
    }
    for ( int i=0; i< newSegmentList.size(); i++) {
        intersects = false;
        for (int j=0; j < list.size(); j++)  {
            if (newSegmentList[i].intersects(list[j]))
                intersects = true;
        }
    for (int j=0; j < doors.size(); j++)  {
        if ((newSegmentList[i].intersects(doors[j])) && ( i != j ))
            intersects = true;
    }
    for (int j=0; j < frontaliere.size(); j++) {
        if ((newSegmentList[i].intersects(frontaliere[j])))
            intersects = true;
    }
        //TODO Check assunzione che newsegmentlist.size = list.size e che siano ordinati assieme.
        if (!intersects) newRoom.addDoor( SemanticMapping::Geometry::DoorLineSegment(doors[i]));
    }

    newSegmentList.clear();

    for ( int i=0; i < frontaliere.size(); i++)
    {
        newSegmentList.append (SLAM::Geometry::LineSegment( p, frontaliere[i].centroid() ) );
    }
    for ( int i=0; i< newSegmentList.size(); i++) {
        intersects = false;
        for (int j=0; j < list.size(); j++)  {
            if (newSegmentList[i].intersects(list[j]))
                intersects = true;
        }
        for (int j=0; j < doors.size(); j++)  {
            if (newSegmentList[i].intersects(doors[j]))
                intersects = true;
        }
        for (int j=0; j < frontaliere.size(); j++) {
            if ((newSegmentList[i].intersects(frontaliere[j])) && ( i != j ))
                intersects = true;
        }
        //TODO Check assunzione che newsegmentlist.size = list.size e che siano ordinati assieme.
        if (!intersects)
            newRoom.addFrontier(SLAM::Geometry::Frontier(frontaliere[i]));
    }

    rooms.append( SemanticMapping::Room( newRoom));
}


void SemanticHandler::MergeRooms()
{
    bool merged[rooms.size()][rooms.size()];
    bool removedSingleRoom[rooms.size()];
    for (int i=0; i< rooms.size(); i++)
    {
        removedSingleRoom[i] = false;
        for (int j=0; j<rooms.size(); j++)
            merged[i][j] = false;
    }
    for (int i=0; i < rooms.size(); i++) {
        for (int j=i; j < rooms.size(); j++) {
            if ( i!=j ){
                SLAM::Geometry::LineSegment connection(rooms[i].getCorePoint(),rooms[j].getCorePoint());
                bool intersection = false;
                foreach (SLAM::Geometry::LineSegment w, map->getWalls())
                {
                    if (connection.intersects(w))
                    {
                        intersection = true;
                        //TODO vaffanculo
                        //break;
                    }
                }
                foreach (Geometry::DoorLineSegment d, doors)
                {
                    if (connection.intersects(d))
                    {
                        intersection = true;
                        //TODO come esco da uno solo?
                        //break;
                    }
                }
                foreach(SLAM::Geometry::LineSegment f, map->getFrontiers())
                {
                    if (connection.intersects(f))
                    {
                        intersection = true;
                        //TODO vaffanculo
                        //break;
                    }
                }
                if (!intersection)
                {
                    merged[j][i] = true;
                }
            }
        }
    }
    for (int i=rooms.size()-1; i > 0; i--)
    {
        for (int j=i; j > 0; j--)
        {
            if ((merged[i][j]) && (!removedSingleRoom[i]))
            {
                //TODO Debug! Rimuovere
                /*
                ldbg << "sto mergiando le stanze "<< i << " e " << j <<endl;
                ldbg << "Il label è " << rooms[i].getInfo() <<endl;
                ldbg << "L'area è " << rooms[i].getArea() <<endl;
                ldbg << "Lunghezza Corridoio " << rooms[i].getCorridorLenght() <<endl;
                ldbg << "E le porte sono " << rooms[i].getGrade() <<endl;
                QList<SLAM::Geometry::Point> *pointlist = rooms[i].getBorderPoint();
                ldbg << pointlist->at(0) <<endl;
                ldbg << pointlist->at(1) <<endl;
                ldbg << pointlist->at(2) <<endl;
                ldbg << pointlist->at(3) <<endl;
                ldbg << "Graphics [{";
                ldbg << " Red, ";
                foreach (SLAM::Geometry::LineSegment p , rooms[i].getRoom())
                {
                    ldbg << p << "," << endl;
                }
                ldbg << "Blue, ";
                foreach(SemanticMapping::Geometry::DoorLineSegment d, rooms[i].getDoors())
                {
                    ldbg << d << ", " << endl;
                }
                SLAM::Geometry::Point punto = rooms[i].getCorePoint();
                ldbg << "Black, ";
                ldbg << "Line[{" << pointlist->at(0) << "," << pointlist->at(0) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(1) << "," << pointlist->at(1) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(2) << "," << pointlist->at(2) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(3) << "," << pointlist->at(3) << "}],"<<endl;
                ldbg << "Line[{{" << punto.x() << " , "<< punto.y()<< "},{" << punto.x() << " , "<< punto.y()<< "}}], "<<endl;
                ldbg << " Green, ";
                foreach (SLAM::Geometry::LineSegment p , rooms[j].getRoom())
                {
                    ldbg << p << "," << endl;
                }
                ldbg << "Purple, ";
                foreach(SemanticMapping::Geometry::DoorLineSegment d, rooms[j].getDoors())
                {
                    ldbg << d << ", " << endl;
                }
                SLAM::Geometry::Point punto2 = rooms[j].getCorePoint();
                ldbg << "Black, ";
                ldbg << "Line[{" << pointlist->at(0) << "," << pointlist->at(0) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(1) << "," << pointlist->at(1) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(2) << "," << pointlist->at(2) << "}],"<<endl;
                ldbg << "Line[{" << pointlist->at(3) << "," << pointlist->at(3) << "}],"<<endl;
                ldbg << "Orange, ";
                ldbg << "Line[{{" << punto2.x() << " , "<< punto2.y()<< "},{" << punto2.x() << " , "<< punto2.y()<< "}}] ";
                ldbg << "}]" <<endl;
*/
                //TODO fineDebug,Rimuovere qui sopra!
                rooms[j].addOtherRoom(rooms[i]);
                removedSingleRoom[i] = true;
            }
        }
    }
    for (int i=rooms.size(); i > 0; i--)
        if (removedSingleRoom[i])
            rooms.removeAt(i);
}
void SemanticHandler::RemoveRooms()
//TODO CHECK sta funzione
{   bool removed[rooms.size()];
    for (int i=0; i< rooms.size(); i++) removed[i] = false;
    for (int i=0; i < rooms.size(); i++) {
        QList<SLAM::Geometry::LineSegment> list = rooms[i].getRoom();
        foreach (SLAM::Geometry::LineSegment u, list)
        {
            bool near = false;
            foreach(SLAM::Geometry::LineSegment d, list)
            {
                if ((d != u) && (d.distance( u.centroid()) < FAR_SEGMENT))
                        near = true;
            }
            if (!near)
            {
                rooms[i].removeSegment(u);
            }
        }
        if (list.size() < MIN_ROOM_SEGMENT)
        {
            removed[i] = true;
        }
    //TODO Rimuovere il fatto che quando trovo "true" vado avanti nella ricerca delle cose
    }
    for (int i=rooms.size()-1; i > 0; i--)
        if (removed[i]){
            //TODO anche usare il distruttore!
            //delete rooms[i];
    //TODO Rimuovere qui sotto, debug!
            /*
    ldbg << "sto rimuovendo la stanza "<< i <<endl;
    ldbg << "Il label è " << rooms[i].getInfo() <<endl;
    ldbg << "L'area è " << rooms[i].getArea() <<endl;
    ldbg << "Lunghezza Corridoio " << rooms[i].getCorridorLenght() <<endl;
    ldbg << "E le porte sono " << rooms[i].getGrade() <<endl;
    QList<SLAM::Geometry::Point> *pointlist = rooms[i].getBorderPoint();
    ldbg << pointlist->at(0) <<endl;
    ldbg << pointlist->at(1) <<endl;
    ldbg << pointlist->at(2) <<endl;
    ldbg << pointlist->at(3) <<endl;
    ldbg << "Graphics [{";
    ldbg << " Red, ";
    foreach (SLAM::Geometry::LineSegment p , rooms[i].getRoom())
    {
        ldbg << p << "," << endl;
    }
    ldbg << "Blue, ";
    foreach(SemanticMapping::Geometry::DoorLineSegment d, rooms[i].getDoors())
    {
        ldbg << d << ", " << endl;
    }
    SLAM::Geometry::Point punto = rooms[i].getCorePoint();
    ldbg << "Black, ";
    ldbg << "Line[{" << pointlist->at(0) << "," << pointlist->at(0) << "}],"<<endl;
    ldbg << "Line[{" << pointlist->at(1) << "," << pointlist->at(1) << "}],"<<endl;
    ldbg << "Line[{" << pointlist->at(2) << "," << pointlist->at(2) << "}],"<<endl;
    ldbg << "Line[{" << pointlist->at(3) << "," << pointlist->at(3) << "}],"<<endl;
    ldbg << "Line[{{" << punto.x() << " , "<< punto.y()<< "},{" << punto.x() << " , "<< punto.y()<< "}}]";
    ldbg << "}]" <<endl;
    */
    //TODO Rimuovere qui sopra!
            rooms.removeAt(i);
        }

}

void SemanticHandler::ProcessRooms()
{
    for (int i=0; i< rooms.size(); i++)
    {
        if (!rooms[i].roomHandled())
                rooms[i].handleRoom();
    }

}
}
