#ifndef SEMANTICDEF_H
#define SEMANTICDEF_H
#include "math.h"
namespace SemanticMapping
{
    //Tempo per cui i dati che si hanno dalla mappa sono considerati antiquati;
    #define MAX_TIME_INTERVAL
    // Se un segmento  pi piccolo di cos“ lo scarto,  rumore
    #define MIN_LENGHT_SEGMENT 0.2
    //TODO sono pareti se l'angolo è al più diverso di 10°->10° è un buon valore?
    #define WALL_ANGLE_TOLERANCE M_PI/18
    //TODO Anche sti due valori di tolleranza vanno modificati...:)
    #define M_TOLERANCE 0.2
    #define QM_TOLERANCE 1
    #define X_VERTICAL_TOLERANCE 0.5
    //Number of points created for each square meter in order to find rooms.
    #define POINTS_PER_SQUARE_METER 5
    //Maximum distance between segment considered as consecutive segments
    #define CONSEC_THRESHOLD 0.3
    //Max distance between non consecutive segment where can be inferred the presence of a door
    #define DOOR_THRESHOLD 4
    //Percentuali di segmenti in una stanza che, se uguale a quelli di un altra stanza, le due stanze sono uguali;
    #define ROOM_COMPARISON_PERCENT 0.7
    //Intorno del centroide per cui due stanze sono in realtà la stessa
    #define ROOM_CENTROID_INTERVAL 0.5
    //Angolo per cui 90-angolo = il segmento è verticale
    #define ANGLE_THRESHOLD M_PI/18
    //I seguenti 5 valori sono quelli del classificatore "base" della stanza;
    // rapporto base/altezza stanza per essere un corridoio
    #define ROOM_CORRIDOR_THRESHOLD 0.5
    // stanza grossa = sempre hall
    #define HALL_THRESHOLD 60
    //secondo threshold di grandezza, che serve a decidere se è un corridoio o una hall
    #define CORRIDOR_HALL_THRESHOLD 30
    // terzo threshold di grandezza che serve a catalogare il resto;
    #define BIG_SMALL_ROOM_THRESHOLD 15
    // ora il numero di porte che dicono se ci son tante porte in una stanza;
    #define DOOR_NUMBER 2
    // questo è il threshold che serve a controllare che alla stanza non sia stato aggiungo un segmento troppo lontano per
    //via di una informazione parziale;
    #define FAR_SEGMENT 5
    //se ho meno di questi segmenti, non sono una stanza.
    #define MIN_ROOM_SEGMENT 5
    //il segmento perpendicolare alla porta che viene restituito al path planner è lungo
    #define DOOR_P_SEGMENT 2
    enum RoomCat {SMALL,BIG,CORRIDOR,HALL,ELSE};
    typedef struct semanticInfos
    {
        RoomCat label;
        int area;
        int corridor_lenght;
        int grade;
    } semanticInfos;
}
#endif // SEMANTICDEF_H
