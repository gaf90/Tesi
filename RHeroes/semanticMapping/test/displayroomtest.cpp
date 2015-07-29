#include "displayroomtest.h"
#include "semanticMapping/semantichandler.h"
#include "semanticMapping/room.h"
#include "shared/logger.h"
#include "semanticMapping/test/createsingleroomtest.h"
#include "slam/geometry/linesegment.h"
#include "semanticMapping/SemanticDef.h"

namespace SemanticMapping{
namespace Test{

DisplayRoomTest::DisplayRoomTest(QObject *parent) :
    QObject(parent)
{
    bbbbb = 0;
}

DisplayRoomTest::~DisplayRoomTest()
{
}

void DisplayRoomTest::luposFirstTest(SLAM::Map map)
{
    bool sonoscemo = true;
    int verde = 150;
    int rosso = 30;
    int giallo = 160;
    int blu = 30;
    ldbg << bbbbb <<endl;
    bbbbb++;
     if (bbbbb%50 != 15) return;
    SemanticMapping::RoomCat label;
    SemanticMapping::SemanticHandler testhandler(&map);
    QList <SemanticMapping::Room> list = testhandler.getRooms();
    if((bbbbb%50 == -15)&&(sonoscemo)) {
        ldbg << "ciao, ho in mano la mappa numero "<< bbbbb << endl;
        ldbg << "verde = piccola, rosso = grande, giallo = corridoio, blu = hall" <<endl;
        ldbg << "ora la stampo" <<endl;
        ldbg << "Graphics [{";
        foreach( SemanticMapping::Room r, list){
        QList <SLAM::Geometry::LineSegment> leparetidellastanza = r.getRoom();
        label = r.getInfo();
        if ( label == SMALL )
        {   double ve = double (verde)/250 ;
            ldbg << " RGBColor[0.4 , " << ve << " , 0.2 ], ";
            verde +=10;
            if (verde > 250) verde = 150;
        }
        else {
            if ( label == BIG)
            {
                ldbg << " RGBColor[0.95, " << ((double) rosso )/250 << " ," << (double) rosso /250 <<" ], ";
                rosso +=10;
                if (rosso > 90) verde = 30;
            }
            else {
                if (label == CORRIDOR)
                {
                    ldbg << " RGBColor[0.95, " <<  (double) (giallo/250) << " , 0.3 ], ";
                    giallo +=10;
                    if (giallo > 250) giallo = 150;
                }
                else {
                    if (label == HALL)
                    {
                        ldbg << " RGBColor[0, " << (double) blu/250 << " , 0.9 ], ";
                        blu +=10;
                        if (blu > 150) blu = 30;
                    }
                    else
                        ldbg << " RGBColor[0.6,  0.5 , 0.9 ], ";
                }
            }
        }
        foreach (SLAM::Geometry::LineSegment p, leparetidellastanza){
            ldbg << p << " , " << endl;
            }
        }
        ldbg << "Line[{{0,0},{0,0}}]";
        ldbg << "}]" <<endl;

        for (int k = 0; k < 3; k++)
        {
            int numeroacaso = rand() / (RAND_MAX + 1.0) * (list.size());
            ldbg << "Il label è " << list[numeroacaso].getInfo() <<endl;
            ldbg << "L'area è " << list[numeroacaso].getArea() <<endl;
            ldbg << "Lunghezza Corridoio " << list[numeroacaso].getCorridorLenght() <<endl;
            ldbg << "E le porte sono " << list[numeroacaso].getGrade() <<endl;
            QList<SLAM::Geometry::Point> *pointlist = list[numeroacaso].getBorderPoint();
            ldbg << pointlist->at(0) <<endl;
            ldbg << pointlist->at(1) <<endl;
            ldbg << pointlist->at(2) <<endl;
            ldbg << pointlist->at(3) <<endl;
            ldbg << "Graphics [{";
            ldbg << " Red, ";
            foreach (SLAM::Geometry::LineSegment p , list[numeroacaso].getRoom())
            {
                ldbg << p << "," << endl;
            }
            ldbg << "Blue, ";
            foreach(SemanticMapping::Geometry::DoorLineSegment d, list[numeroacaso].getDoors())
            {
                ldbg << d << ", " << endl;
            }
            ldbg << "Orange, ";
            foreach (SLAM::Geometry::LineSegment f , list[numeroacaso].getFrontiers())
            {
                ldbg << f << "," << endl;
            }
            SLAM::Geometry::Point punto = list[numeroacaso].getCorePoint();
            ldbg << "Black, ";
            ldbg << "Line[{" << pointlist->at(0) << "," << pointlist->at(0) << "}],"<<endl;
            ldbg << "Line[{" << pointlist->at(1) << "," << pointlist->at(1) << "}],"<<endl;
            ldbg << "Line[{" << pointlist->at(2) << "," << pointlist->at(2) << "}],"<<endl;
            ldbg << "Line[{" << pointlist->at(3) << "," << pointlist->at(3) << "}],"<<endl;
            ldbg << "Line[{{" << punto.x() << " , "<< punto.y()<< "},{" << punto.x() << " , "<< punto.y()<< "}}]";
            ldbg << "}]" <<endl;
        }
    }
    if ((bbbbb%50 == 15)&&(sonoscemo)) {
        ldbg << "ciao, ho in mano la mappa numero "<< bbbbb << endl;
        ldbg << "ora la stampo" <<endl;
        ldbg << "Graphics [{";
        ldbg << "Black,";
        foreach(SLAM::Geometry::LineSegment p, map.getWalls()){
            ldbg << p <<", " <<endl;
        }
        QList<SemanticMapping::Geometry::DoorLineSegment> porte = testhandler.getDoors();
        ldbg << "Blue, ";
        foreach(SemanticMapping::Geometry::DoorLineSegment d, porte)
        {
            ldbg << d << ", " <<endl;
        }
        ldbg << "Orange, ";
        foreach(SLAM::Geometry::LineSegment f, map.getFrontiers())
        {
            ldbg << f << ", " <<endl;
        }
        ldbg << "Line[{{0,0},{0,0}}]";
        ldbg << "}]";
    }
}


}
}
