#include "room.h"


namespace SemanticMapping{

Room::Room(QList<SLAM::Geometry::LineSegment> &prevRoom, SLAM::Geometry::Point &init):
    corepoint(init), walls(prevRoom)
{
}
Room::Room(SLAM::Geometry::Point &init):
    corepoint(init)
{
}

Room::Room(const Room &r):
    finished(r.finished), walls(r.walls), frontiers(r.frontiers), doors(r.doors), area(r.area),
    corridorLenght(r.corridorLenght), n_o(r.n_o), n_e(r.n_e), s_o(r.s_o),
    s_e(r.s_e), corepoint(r.corepoint), centroid(r.centroid), roomlabel(r.roomlabel)
{
}

//TODO
Room::~Room()
{
//    delete corepoint;
//    foreach(SLAM::Geometry::LineSegment *s, walls)
//        delete s;
//    foreach(SemanticMapping::Geometry::DoorLineSegment *s, doors)
//        delete s;
}

//TODO aggiunge il segmento alla stanza
void Room::addSegment(SLAM::Geometry::LineSegment newseg)
{
    walls.append(newseg);
}

void Room::addDoor(Geometry::DoorLineSegment newdoor)
{
    doors.append(newdoor);
}

void Room::addFrontier(SLAM::Geometry::Frontier newfrontier)
{
    frontiers.append(newfrontier);
}

void Room::addOtherRoom(const Room &r)
{
    bool found = false;
    foreach(SLAM::Geometry::LineSegment l1, r.walls)
    {
        found = false;
        foreach(SLAM::Geometry::LineSegment l2, walls)
            if (l1 == l2) found = true;
        if (!found)
            walls.append(l1);
    }
    //Ci saranno mai due porte uguali?
    foreach(SemanticMapping::Geometry::DoorLineSegment l1, r.doors)
    {
        found = false;
        foreach(SemanticMapping::Geometry::DoorLineSegment l2, doors)
            if (l1 == l2) found = true;
        if (!found)
            doors.append(l1);
    }

    foreach(SLAM::Geometry::Frontier l1, r.frontiers)
    {
        found = false;
        foreach(SLAM::Geometry::Frontier l2, frontiers)
            if (l1 == l2) found = true;
        if (!found)
            frontiers.append(l1);
    }

    handleRoom();
}

void Room::removeSegment(SLAM::Geometry::LineSegment wrong)
{
    int i = -1;
    foreach(SLAM::Geometry::LineSegment s, walls)
    {
        int j=0;
        if (s == wrong) i=j;
        i++;
    }
    if (i != -1) walls.removeAt(i);
}
//TODO Ritorno la stanza, o meglio "walls"
const QList<SLAM::Geometry::LineSegment> &Room::getRoom()
{
    return walls;
}

const QList<SemanticMapping::Geometry::DoorLineSegment> &Room::getDoors()
{
    return doors;
}

const QList<SLAM::Geometry::Frontier> &Room::getFrontiers()
{
    return frontiers;
}

bool Room::isInRoom( SLAM::Geometry::Point &p)
{
    double x = p.x();
    double y = p.y();
    //TODO Chec' se la funzione gestisce NO NE SO SE correttamente, e sarebbe più carino implementarla meglio.
    if ((x > n_o.x()) && (x < n_e.x()) && (x > s_o.x()) && ( x < s_e.x()) &&
            (y > n_o.y()) && (y < n_e.y()) && (y > s_o.y()) && ( y < s_e.y()))
        return true;
    else return false;
}

RoomCat Room::getInfo()
{
    return roomlabel;
}

double Room::getArea()
{
    return area;
}

int Room::getGrade()
{
    //TODO: Qui ritorno il numero di porte; Potrei implementare una modifica che conta effettivamente il # di stanze;
    return doors.length();
}

SLAM::Geometry::Point Room::getCorePoint()
{
    return corepoint;
}

double Room::getCorridorLenght()
{
    return corridorLenght;
}

QList<SLAM::Geometry::Point> *Room::getBorderPoint()
{
    QList<SLAM::Geometry::Point> *pointList = new QList<SLAM::Geometry::Point>();
    pointList->append(n_o);
    pointList->append(s_o);
    pointList->append(s_e);
    pointList->append(n_e);
    return pointList;
}

SLAM::Geometry::Point *Room::getCentroid()
{
    if (!Room::finished) Room::handleRoom();
    SLAM::Geometry::LineSegment d1(n_e, s_o);
    SLAM::Geometry::LineSegment d2(n_o, s_e);
    centroid = d1.intersection(d2);
    return new SLAM::Geometry::Point(centroid);
}


bool Room::roomHandled()
{
    return finished;
}

bool Room::handleRoom()
{
    bool isTrue = false;
    bool swapped = false;
    double distance1, distance2, angle;
    //TODO classifico qui? classifico qui...devo quindi allocare la struttura dati dell'istogramma!
    //TODO devo calcolare anche NO, NE, SO, SE
    n_o = corepoint;
    s_o = corepoint;
    n_e = corepoint;
    s_e = corepoint;
    //TODO non è un metodo molto furbo per trovare i 4 vertici, però non mi viene in mente molto altro. cmq va modificato
    //il centroide non va bene.
    for (int i=0; i < walls.size(); i++)
    {
//        if ((walls[i].vertexMinX().x() < n_o.x()) && (walls[i].vertexMaxY().y() > n_o.y()))
//                n_o = walls[i].centroid();
//        if ((walls[i].vertexMinX().x() < s_o.x()) && (walls[i].vertexMinY().y() < s_o.y()))
//                s_o = walls[i].centroid();
//        if ((walls[i].vertexMaxX().x() > n_e.x()) && (walls[i].vertexMaxY().y() > n_e.y()))
//                n_e = walls[i].centroid();
//        if ((walls[i].vertexMaxX().x() > s_e.x()) && (walls[i].vertexMinY().y() < s_e.y()))
//                s_e = walls[i].centroid();
        //TODO CAMBIO NO = NORD SO = OVEST NE = EST SE = SUD, se il metodo funziona allora devo rinominare variabili
        if (walls[i].vertexMaxY().y() > n_o.y())
                n_o = walls[i].vertexMaxY();
        if (walls[i].vertexMinX().x() < s_o.x())
                s_o = walls[i].vertexMinX();
        if (walls[i].vertexMaxX().x() > n_e.x())
                n_e = walls[i].vertexMaxX();
        if (walls[i].vertexMinY().y() < s_e.y())
                s_e = walls[i].vertexMinY();
    }

    //Da qui in poi è codice copiato e risistemato da semantichandler.cpp
    QList<initWallStruct> consecutive;
    QList<secondWallStruct> secondList;
    QList<thirdWallStruct> thirdList;

    for (int i = 0; i < walls.size(); ++i)  {
        //Se Ë vuota consecutive, appendo il primo.
        //TODO Cambiato qui
        if ( walls[i].length() > MIN_LENGHT_SEGMENT) {
        if (consecutive.size() == 0) {
            QList<SLAM::Geometry::LineSegment> prova;
            double angoloTemp = fabs(fmod (walls[i].angle() + M_PI ,M_PI ));
            while (angoloTemp >= M_PI - 2*WALL_ANGLE_TOLERANCE)
                angoloTemp -= M_PI;
            Room::initWallStruct temp = { angoloTemp , prova, false, false, 0 };
            temp.line.append(walls[i]);
            consecutive.append( temp );
        }
        else
        {
            isTrue = false;
            for (int j = 0; j < consecutive.size(); ++j)   {
                //TODO non so se sta roba funziona, l'intero if, ma sicuramente va cambiato.
                if ((( fabs(fmod(walls.at(i).angle() + M_PI, M_PI )) >= consecutive[j].angle - WALL_ANGLE_TOLERANCE) &&
                        ( fabs(fmod(walls.at(i).angle() + M_PI, M_PI ))  <= consecutive[j].angle + WALL_ANGLE_TOLERANCE))||
                        (( fabs(fmod(walls.at(i).angle() + M_PI, M_PI )) >= M_PI - 2*WALL_ANGLE_TOLERANCE) &&
                        (consecutive[j].angle < 2*WALL_ANGLE_TOLERANCE)))
                {
                    if (( fabs(fmod(walls.at(i).angle() + M_PI, M_PI )) >= M_PI - 2*WALL_ANGLE_TOLERANCE) &&
                        (consecutive[j].angle < 2*WALL_ANGLE_TOLERANCE))
                            consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(walls[i].angle() + M_PI, M_PI)) - M_PI) /
                                (consecutive[j].line.size() + 1);
                    else {
                    //Aggiorno l'angolo della struttura,
                    //TODO  Qui, come gestico la struttura? farlo in maniera pesata!
                    consecutive[j].angle = (consecutive[j].angle * consecutive[j].line.size() + fabs(fmod(walls[i].angle() + M_PI, M_PI))) /
                            (consecutive[j].line.size() + 1);}
                    //Aggiungo la linea alla struttura.
                    consecutive[j].line.append(walls.at(i));
                    isTrue = true;
                }
            }
            //Nessuna parete trovata = nuova parete!
            if (!isTrue) {
                QList<SLAM::Geometry::LineSegment> prova;
                double angoloTemp = fabs(fmod (walls[i].angle() + M_PI ,M_PI ));
                while (angoloTemp >= M_PI - 2*WALL_ANGLE_TOLERANCE)
                    angoloTemp -= M_PI;
                Room::initWallStruct temp = { angoloTemp , prova, false, false, 0 };
                temp.line.append(walls[i]);
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
    for (int z = 0; z< consecutive.size(); z++)  {
        if( z != 0)
        {
        thirdList.append(roomClassifierHelper(secondList));

        }
        secondList.clear();
        for (int i = 0; i < consecutive.at(z).line.size(); ++i)  {
            //Se è vuota consecutive, appendo il primo.
            if ( secondList.size() == 0 ) {
                QList<SLAM::Geometry::LineSegment> altraProva;
                Room::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
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
                    Room::secondWallStruct temp2 = {consecutive.at(z).line.at(i).m(),
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
    //TODO errore qui!
    if (secondList.size()>=1)
        thirdList.append(roomClassifierHelper(secondList));
    //Anche qui
    if (thirdList.size() == 0) return false;
    do{
        swapped = false;
        for (int j = 1; j < thirdList.size(); j++ ) {
            if (thirdList[j-1].lenght > thirdList[j].lenght) {
            thirdList.swap( j, j-1 );
            swapped = true;
            }
        }
   }
        while(swapped);
    distance1 = thirdList[0].lenght;
    distance2 = 0;
    angle = thirdList[0].angle;
    for (int i = 1; i < thirdList.size(); i++) {
        if (( angle <= thirdList[i].angle - WALL_ANGLE_TOLERANCE) &&
                (angle >= thirdList[i].angle + WALL_ANGLE_TOLERANCE))
        {
            distance2 = thirdList[i].lenght;
            break;
        }
    }
    //TODO Sto classificando qui, sta cosa di distance1 e 2 non mi piace, va rivista;
    corridorLenght = distance1;
    calcArea();
    classifyRoom(distance1, distance2);
    finished=true;
//    for (int i=thirdList.size(); i > 0; i--)
//    {
//        delete thirdList[i];
//    }
    return finished;
}

//TODO qui cosa diavolo facevo? Segnarlo da qualche parte!
Room::thirdWallStruct Room::roomClassifierHelper( QList<secondWallStruct> list )
{
    double angle = 0;
    bool swapped = false;
    bool orizontal = false;
    double distance = 0;
    double maxDistance = 0;
    for (int i = 0; i < list.size(); i++ )
    {
        orizontal = false;
        if ((list[i].line[0].angle() <= M_PI/4) || (list[i].line[0].angle() >= 3*M_PI/4)) {
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
    }
    distance = 0;
    maxDistance = 0;
    for (int i = 0; i < list.size(); i++ )
    {
        orizontal = false;
        if ((list[i].line[0].angle() <= M_PI/4) || (list[i].line[0].angle() >= 3/4 * M_PI))
            orizontal = true;
        if(orizontal)
            distance = list[i].line.first().vertexMinX().distance(list[i].line.last().vertexMaxX());
        else distance = list[i].line.first().vertexMinY().distance(list[i].line.last().vertexMaxY());
        maxDistance += distance;
    }
    //TODO non è molto corretto qui, ma vabbè. Controllare il for seguente;
    if (list.size() == 0)
        ldbg << "C'è un problema qui!" <<endl;
    else {
    for (int i = 0; i < list[0].line.size(); i++) {
        angle += list[0].line[i].angle();
    }
    angle = angle / list[0].line.size();}
    Room::thirdWallStruct temp = {angle, maxDistance}; /* new Room::thirdWallStruct;
    temp->angle = angle;
    temp->lenght = maxDistance;*/
    //TODO come passo?
    return temp;

}

void Room::classifyRoom(double d1, double d2) {
    /*
    Spiegone del primo metodo di prova per classificare:
        A: #porte > thrashold
        B: rapporto tra i due lati maggiori (d1 e d2) maggiore di un threshold (stanza molto rettangolare o molto quadrata)
        C: Area abbastanza grossa;
        D: Area Molto grossa;
     Sono 5 regole in cascata, una volta attivata una le altre vengono ignorate (sono in ordine di priorità):
        1) D -> HALL
        2) (A v B) ^ C ->HALL;
        3) (A v B) -> CORRIDOR;
        4) C -> BIG;
        5) else ->SMALL;
    */
    double r =  d1 > d2 ? d2/d1 : d1/d2;
    if (area > HALL_THRESHOLD)
    {
        roomlabel = HALL;
    }
    else {
        if (((r > ROOM_CORRIDOR_THRESHOLD ) || (doors.size() > DOOR_NUMBER))
            && ( area > CORRIDOR_HALL_THRESHOLD))
        {
            roomlabel = HALL;
        }
        else {
            if ((r > ROOM_CORRIDOR_THRESHOLD ) || (doors.size() > DOOR_NUMBER))
            {
                roomlabel = CORRIDOR;
            }
            else {
                if (area > BIG_SMALL_ROOM_THRESHOLD)
                {
                    roomlabel = BIG;
                }
                else {
                    roomlabel = SMALL;
                }
            }
        }
    }
}

void Room::calcArea()
{
    // Area = 1/2 ||AC x BD|| with AC and BD diagonals of the quadrilateral;
    double x1,x2,y1,y2;
    x1 = n_e.x() - s_o.x() ;
    x2 = n_o.x() - s_e.x() ;
    y1 = n_e.y() - s_o.y() ;
    y2 = n_o.y() - s_e.y() ;
    area = 0.5 * fabs(x1*y2-x2*y1);
}

}
