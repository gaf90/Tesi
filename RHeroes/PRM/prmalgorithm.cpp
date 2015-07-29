#include "prmalgorithm.h"
#include <QTime>
#include <QElapsedTimer>
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include "exploration/explorationconstants.h"

namespace PRM {

using namespace SLAM::Geometry;
using namespace SLAM;

PRMAlgorithm::PRMAlgorithm(): QThread(), pathFolder("Script"),
    graph(Graph()), oldMap(), iterationNumber(0), aStar(NULL), frontierID(0), stateMutex(new QMutex()),
    currentPath(QList<Point*>()), plotTimer(new QTimer(this)){
    startTime = QTime::currentTime();
    qsrand((uint)startTime.msec());
    color << "'b'" <<"'r'" <<"'m'" <<"'g'";
    //Crea directory
    if(!QDir("Matlab").exists()){
        QDir().mkdir("Matlab");
    }
    fullPath = "Matlab/";
    fullPath.append(pathFolder);
    if(!QDir(fullPath).exists()){
        QDir().mkdir(fullPath);
    }
    dataPath=fullPath;
    dataPath.append("/Data");
    if(!QDir(dataPath).exists()){
        QDir().mkdir(dataPath);
    }
    //distruzione script nella directory
    QDir dir(fullPath);
    foreach(QString dirFile,dir.entryList()){
        dir.remove(dirFile);
    }
    QDir dir2(dataPath);
    foreach(QString dirFile,dir2.entryList()){
        dir2.remove(dirFile);
    }
    connect(this,SIGNAL(sigStartTimerPP()),this,SLOT(onStartTimerPP()),Qt::QueuedConnection);
    connect(this,SIGNAL(sigStopTimerPP()),this,SLOT(onStopTimer()),Qt::QueuedConnection);
    connect(plotTimer,SIGNAL(timeout()),this,SLOT(onTimeout()));
    moveToThread(this);
    ldbg << "PRM Algorithm: initialized" << endl;
    start();
}

PRMAlgorithm::PRMAlgorithm(QString folder): QThread(), pathFolder(folder),
    graph(Graph()), oldMap(), iterationNumber(0), aStar(NULL), frontierID(0), stateMutex(new QMutex()), currentPath(QList<Point*>()), plotTimer(new QTimer(this)){
    startTime = QTime::currentTime();
    qsrand((uint)startTime.msec());
    color << "'b'" <<"'r'" <<"'m'" <<"'g'";
    //Crea directory
    if(!QDir("Matlab").exists()){
        QDir().mkdir("Matlab");
    }
    fullPath = "Matlab/";
    fullPath.append(pathFolder);
    if(!QDir(fullPath).exists()){
        QDir().mkdir(fullPath);
    }
    dataPath=fullPath;
    dataPath.append("/Data");
    if(!QDir(dataPath).exists()){
        QDir().mkdir(dataPath);
    }
    //distruzione script nella directory
    QDir dir(fullPath);
    foreach(QString dirFile,dir.entryList()){
        dir.remove(dirFile);
    }
    QDir dir2(dataPath);
    foreach(QString dirFile,dir2.entryList()){
        dir2.remove(dirFile);
    }
    connect(this,SIGNAL(sigStartTimerPP()),this,SLOT(onStartTimerPP()),Qt::QueuedConnection);
    connect(this,SIGNAL(sigStopTimerPP()),this,SLOT(onStopTimer()),Qt::QueuedConnection);
    connect(plotTimer,SIGNAL(timeout()),this,SLOT(onTimeout()));
    moveToThread(this);
    ldbg << "PRM Algorithm: initialized" << endl;
    start();
}

PRMAlgorithm::~PRMAlgorithm(){
    emit sigStopTimerPP();
}

void PRMAlgorithm::updatePRM(Map newMap)
{
    stateMutex->lock();
    QElapsedTimer timer;
    timer.start();
    int pointCounter=0;
    if(newMap.getMaxX()==-INFINITY)
    {
        ldbg << "PRM Algorithm: Empty map" << endl;
    }
    else
    {
        const PathNode* newRobotPose = newMap.lastRobotPose(Config::robotID);
        double xRobotNew= newRobotPose->x();
        double yRobotNew= newRobotPose->y();
        if(iterationNumber==0)
        {
            ldbg << "PRM Algorithm: update, iteration: " << iterationNumber << endl;
            frontiersAdded = foundFrontiersAdded(newMap);
            frontiersRemoved = foundFrontiersRemoved(newMap);

            for(int i=0;i<Config::PRM::pointNumber;i++)
            {
                Point* p = newRandomPoint(newMap);
                if(visibilityCheckNewMap(p,newMap)){
                    //ldbg<<"Controllo visibility!!!!"<<endl;
                    graph.addNode(p,newMap);
                    pointCounter++;
                }
            }
            if(Config::PRM::pointNumberFrontier!=0){
                foreach(Frontier f, frontiersRemoved)
                    graph.deleteFrontierNodes(&f);
                foreach(Frontier f, frontiersAdded)
                {
                    Point centroid= f.centroid();

                    double fx1,fy1,fx2,fy2;
                    if(f.x1()<= f.x2()){
                        fx1=f.x1();
                        fy1=f.y1();
                        fx2=f.x2();
                        fy2=f.y2();
                    }
                    else{
                        fx1=f.x2();
                        fy1=f.y2();
                        fx2=f.x1();
                        fy2=f.y1();
                    }
                    double dxFrontier= fx2-fx1;
                    double dyFrontier= fy2-fy1;
                    double angleFrontier = atan2(dyFrontier,dxFrontier);
                    GraphNode* node= graph.nearestNode(&centroid);
                    //punto test sopra
                    double x1= centroid.x()+0.1*cos(angleFrontier+M_PI_2);
                    double y1= centroid.y()+0.1*sin(angleFrontier+M_PI_2);
                    Point p1= Point(x1,y1);
                    //punto test sotto
                    double x2= centroid.x()+0.1*cos(angleFrontier-M_PI_2);
                    double y2= centroid.y()+0.1*sin(angleFrontier-M_PI_2);
                    Point p2= Point(x2,y2);
                    boolplus isAbove=Maybe;
                    if(node!=NULL){
                        if(newMap.isReachable(p1,*(node->getPoint()),0)==1){
                            isAbove=True;
                        }
                        if(newMap.isReachable(p2,*(node->getPoint()),0)==1){
                            isAbove=False;
                        }
                        if(isAbove!=Maybe){
                            for(int i=0;i<Config::PRM::pointNumberFrontier;i++){
                                Point* p = newRandomPointFrontier(&f, newMap,isAbove);
                                if(visibilityCheckFrontier(p,&f,newMap)){
                                    graph.addNode(p,newMap,&f);
                                    pointCounter++;
                                }
                            }
                        }
                    }
                }
            }
            //ldbg << "PRM Algorithm: plot map" << endl;
            if(pointCounter==0){
                //ldbg << "PRM Algorithm: iteration time: " << timer.elapsed()<<"ms. Added: "<<pointCounter<<" nodes"<< endl;
            }
            else{
                //plot(newMap);
                iterationNumber= iterationNumber + 1;
                xRobot=xRobotNew;
                yRobot=yRobotNew;
                oldMap= newMap;
                aStar=NULL;
                ldbg << "PRM Algorithm: iteration time: " << timer.elapsed()<<"ms. Added: "<<pointCounter<<" nodes"<< endl;
            }
            emit sigStartTimerPP();
        }
        else{
            if(!(comapareDouble(xRobot,xRobotNew) && comapareDouble(yRobot,yRobotNew))){
                ldbg << "PRM Algorithm: update, iteration: " <<iterationNumber<<endl;
                bool modified = false;
                frontiersAdded=foundFrontiersAdded(newMap);
                frontiersRemoved=foundFrontiersRemoved(newMap);
                for(int i=0;i<Config::PRM::pointNumber;i++){
                    Point* p = newRandomPoint(newMap);
                    if(visibilityCheck(p,newMap)){
                        graph.addNode(p,newMap);
                        pointCounter++;
                        modified=true;
                    }
                }
                if(Config::PRM::pointNumberFrontier!=0){
                    foreach(Frontier f, frontiersRemoved){
                        graph.deleteFrontierNodes(&f);
                    }
                    foreach(Frontier f, frontiersAdded){

                        Point centroid= f.centroid();

                        double fx1,fy1,fx2,fy2;
                        if(f.x1()<= f.x2()){
                            fx1=f.x1();
                            fy1=f.y1();
                            fx2=f.x2();
                            fy2=f.y2();
                        }
                        else{
                            fx1=f.x2();
                            fy1=f.y2();
                            fx2=f.x1();
                            fy2=f.y1();
                        }
                        double dxFrontier= fx2-fx1;
                        double dyFrontier= fy2-fy1;
                        double angleFrontier = atan2(dyFrontier,dxFrontier);
                        GraphNode* node= graph.nearestNode(&centroid);
                        //punto test sopra
                        double x1= centroid.x()+0.1*cos(angleFrontier+M_PI_2);
                        double y1= centroid.y()+0.1*sin(angleFrontier+M_PI_2);
                        Point p1= Point(x1,y1);
                        //punto test sotto
                        double x2= centroid.x()+0.1*cos(angleFrontier-M_PI_2);
                        double y2= centroid.y()+0.1*sin(angleFrontier-M_PI_2);
                        Point p2= Point(x2,y2);
                        boolplus isAbove=Maybe;
                        if(node!=NULL){
                            if(newMap.isReachable(p1,*(node->getPoint()),0)==1){
                                isAbove=True;
                            }
                            if(newMap.isReachable(p2,*(node->getPoint()),0)==1){
                                isAbove=False;
                            }
                            if(isAbove!=Maybe){
                                for(int i=0;i<Config::PRM::pointNumberFrontier;i++){
                                    Point* p = newRandomPointFrontier(&f, newMap,isAbove);
                                    if(visibilityCheckFrontier(p,&f,newMap)){
                                        graph.addNode(p,newMap,&f);
                                        pointCounter++;
                                        modified=true;
                                    }
                                }
                            }
                        }
                    }
                }
                if(modified){
                    //plot(newMap);
                }
                xRobot=xRobotNew;
                yRobot=yRobotNew;
                oldMap= newMap;
                ldbg << "PRM Algorithm: iteration time: " << timer.elapsed()<<"ms. Added: "<<pointCounter<<" nodes"<< endl<<endl;
                aStar=NULL;
                iterationNumber= iterationNumber + 1;
            }
        }
    }
    frontierID=0;
    stateMutex->unlock();
}

//nuovoPunto
Point* PRMAlgorithm::newRandomPoint(Map newMap){
    const PathNode* robotPose = newMap.lastRobotPose(Config::robotID);
    //distanza
    double randomDistance = qrand() % (Config::PRM::precision-1);    //0-99
    randomDistance=randomDistance/(Config::PRM::precision-1);    //0-1
    double distance = Config::PRM::maxPointDistance*sqrt(randomDistance);
    //angolo
    double randomAngle = qrand() % Config::PRM::precision;
    randomAngle=randomAngle/Config::PRM::precision;
    double angle = randomAngle*2*M_PI;

    double x= robotPose->x()+distance*cos(angle);
    double y= robotPose->y()+distance*sin(angle);
    return new Point(x,y);
}
//nuovoPuntoFrontiera
Point* PRMAlgorithm::newRandomPointFrontier(Frontier* frontier, Map map, boolplus isAbove){
    Point centroid = frontier->centroid();
    double x1,y1,x2,y2;
    if(frontier->x1()<= frontier->x2()){
        x1=frontier->x1();
        y1=frontier->y1();
        x2=frontier->x2();
        y2=frontier->y2();
    }
    else{
        x1=frontier->x2();
        y1=frontier->y2();
        x2=frontier->x1();
        y2=frontier->y1();
    }
    double dxFrontier= x2-x1;
    double dyFrontier= y2-y1;
    double angleFrontier = atan2(dyFrontier,dxFrontier);
    //distanza
    double randomDistance = (qrand() % (Config::PRM::precision-1))+1;    //1-100
    randomDistance=randomDistance/Config::PRM::precision;    //0-1
    //double distance = sqrt(randomDistance)*(frontier->length()/4);
    double distance = sqrt(randomDistance)*1.5;//0.5;
    //angolo
    double randomAngle = qrand() % (Config::PRM::precision-1)+1;
    randomAngle=randomAngle/Config::PRM::precision;
    double angle = randomAngle*M_PI;
    angle=angle+angleFrontier;
    //    if(((x2-x1)*(robot->y()-y1)-(y2-y1)*(robot->x()-x1))<0){
    //        angle=angle+M_PI;
    //    }
    if(isAbove==False){
        angle=angle+M_PI;
    }
    double x= centroid.x()+distance*cos(angle);
    double y= centroid.y()+distance*sin(angle);
    return new Point(x,y);
}
//controlloPunto
bool PRMAlgorithm::visibilityCheck (Point* point, Map newMap){
    const PathNode* currentRobotPose = newMap.lastRobotPose(Config::robotID);
    //qDebug()<<"currentRobotPose"<<currentRobotPose;
    Point current = Point(currentRobotPose->x(), currentRobotPose->y());
    Point previous = Point(xRobot, yRobot);
    LineSegment movement = LineSegment(current, previous);
    LineSegment visibility = LineSegment(current, *point);
    double xLaser=current.x()+0.2085*cos(currentRobotPose->theta()+M_PI_2);
    double yLaser=current.y()+0.2085*sin(currentRobotPose->theta()+M_PI_2);
    Point laserPosition(xLaser,yLaser);
    foreach(Frontier f, frontiersRemoved){
        //nota: controlliamo prima se è raggiungibile....poi vogliamo controllare la frontiera
        if(f.intersects(movement)){
            if(!(f.intersects(visibility))){
                if((newMap.isReachable(*point,laserPosition,Config::PRM::movementRadius))==1){
                    return true;
                }
            }
        }
        else{
            if((f.intersects(visibility))){
                if((newMap.isReachable(*point,laserPosition,Config::PRM::movementRadius))==1){
                    return true;
                }
            }
        }
    }
    return false;
}
//controlloPuntoFrontiera
bool PRMAlgorithm::visibilityCheckFrontier (Point* point, Frontier* frontier, Map newMap){
    //qDebug()<<"point"<<*point;
    //qDebug()<<"frontier"<<*frontier;
    if(newMap.isReachable(*point,frontier->centroid(),0)!=0){
        return true;
    }
    return false;
}

//controllo validità punto sulla nuova mappa
bool PRMAlgorithm::visibilityCheckNewMap (Point* point, Map newMap){
    const PathNode* currentRobotPose = newMap.lastRobotPose(Config::robotID);
    Point current = Point(currentRobotPose->x(), currentRobotPose->y());
    double xLaser=current.x()+0.2085*cos(currentRobotPose->theta()+M_PI_2);
    double yLaser=current.y()+0.2085*sin(currentRobotPose->theta()+M_PI_2);
    Point laserPosition(xLaser,yLaser);
    //ldbg<<"current"<<current<<endl;
    bool res=(newMap.isReachable(*point,laserPosition,0)==1);
    //ldbg<<"res"<<res<<endl;
    return res;
}

QList<Frontier> PRMAlgorithm::foundFrontiersRemoved(Map newMap){
    QList<Frontier*> oldFrontiers = oldMap.frontiers();
    QList<Frontier*> newFrontiers = newMap.frontiers();
    QList<Frontier> difference;
    fforeach(Frontier *fo, oldFrontiers){
        bool matchFound = false;
        fforeach(Frontier *fn, newFrontiers ){
            Point oldCentroid = fo->centroid();
            Point newCentroid = fn->centroid();
            if(oldCentroid.distance(newCentroid)==0){
                matchFound = true;
                break;
            }
        }
        if (!matchFound){
            difference.append(Frontier(fo->x1(),fo->y1(),fo->x2(),fo->y2()));
        }
    }
    return difference;
}

QList<Frontier> PRMAlgorithm::foundFrontiersAdded(Map newMap){
    QList<Frontier*> oldFrontiers = oldMap.frontiers();
    QList<Frontier*> newFrontiers = newMap.frontiers();
    QList<Frontier> difference;
    fforeach(Frontier *fn, newFrontiers){
        bool matchFound = false;
        fforeach(Frontier *fo, oldFrontiers){
            Point oldCentroid = fo->centroid();
            Point newCentroid = fn->centroid();
            if(oldCentroid.distance(newCentroid)==0){
                matchFound = true;
                break;
            }
        }
        if (!matchFound){
            if(fn->length()>Config::PRM::movementRadius){
                difference.append(Frontier(fn->x1(),fn->y1(),fn->x2(),fn->y2()));
            }
        }
    }
    return difference;
}

void PRMAlgorithm::plot(Map map){
    QString filename= fullPath;
    int seconds=startTime.elapsed()/1000;
    filename.append(QString("/script_%1_map.m").arg(seconds));
    QFile* output= new QFile(filename);
    if(!output->open(QIODevice::WriteOnly | QIODevice::Text)){
        qDebug()<<"ERRORE. Impossibile creare il file!!!";
    }
    else{
        QTextStream out(output);
        out<<"close all; clear all; clc;"<<endl;
        out<<"%graphics_toolkit('gnuplot');"<<endl;
        out<<"f=figure(1);"<<endl;
        out<<"hold on;"<<endl;

        //stampa muri
        QString xWallsIni= "";
        QString yWallsIni= "";
        QString xWallsFin= "";
        QString yWallsFin= "";
        foreach(LineSegment* w, map.walls()){
            xWallsIni.append(" ").append(QString::number(w->x1(),'f',4));
            yWallsIni.append(" ").append(QString::number(w->y1(),'f',4));
            xWallsFin.append(" ").append(QString::number(w->x2(),'f',4));
            yWallsFin.append(" ").append(QString::number(w->y2(),'f',4));
        }
        out<<"xWalls=["<<xWallsIni<<"; "<< xWallsFin <<"];"<<endl;
        out<<"yWalls=["<<yWallsIni<<"; "<< yWallsFin <<"];"<<endl;
        out<<"walls = plot(xWalls, yWalls, 'k');"<<endl;

        //stampa frontiere
        QString xFrontiersIni= "";
        QString yFrontiersIni= "";
        QString xFrontiersFin= "";
        QString yFrontiersFin= "";
        foreach(LineSegment* f, map.frontiers()){
            xFrontiersIni.append(" ").append(QString::number(f->x1(),'f',4));
            yFrontiersIni.append(" ").append(QString::number(f->y1(),'f',4));
            xFrontiersFin.append(" ").append(QString::number(f->x2(),'f',4));
            yFrontiersFin.append(" ").append(QString::number(f->y2(),'f',4));
        }
        out<<"xFrontiers=["<<xFrontiersIni<<"; "<< xFrontiersFin <<"];"<<endl;
        out<<"yFrontiers=["<<yFrontiersIni<<"; "<< yFrontiersFin <<"];"<<endl;
        out<<"frontiers = plot(xFrontiers, yFrontiers, '-.r');"<<endl;

        //stampa grafo
        graph.plot(&out);

        //dimensione plot
        int maxX = map.getMaxX();
        int maxY = map.getMaxY();
        int minX = map.getMinX();
        int minY = map.getMinY();
        out<<"axis(["<<minX-1<<" "<<maxX+1<<" "<<minY-1<<" "<<maxY+1<<"]);"<<endl;

        //stampa path seguito
        QString xPath = "", yPath= "";
        foreach(TimedPose t, map.getRobotPath(Config::robotID)){
            xPath.append(" ").append(QString::number(t.x(),'f',4));
            yPath.append(" ").append(QString::number(t.y(),'f',4));
        }
        out<<"xPath =["<<xPath<<"];"<<endl;
        out<<"yPath =["<<yPath<<"];"<<endl;
        out<<"path = plot(xPath"<<", yPath"<<",'b','LineWidth',2);"<<endl;

        //stampa path da seguire
        if(!currentPath.empty()){
            QString xPathCurrent = "", yPathCurrent= "";
            foreach(Point* p, currentPath){
                xPathCurrent.append(" ").append(QString::number(p->x(),'f',4));
                yPathCurrent.append(" ").append(QString::number(p->y(),'f',4));
            }
            out<<"xPathCurrent =["<<xPathCurrent<<"];"<<endl;
            out<<"yPathCurrent =["<<yPathCurrent<<"];"<<endl;
            out<<"pathCurrent = plot(xPathCurrent, yPathCurrent"<<",'-.r','LineWidth',2);"<<endl;
        }

        //stampa posizione robot
        QString xRobot = "", yRobot = "";
        xRobot.append(QString::number(map.lastRobotPose(Config::robotID)->x(),'f',4));
        yRobot.append(QString::number(map.lastRobotPose(Config::robotID)->y(),'f',4));
        out<<"xRobot = "<<xRobot<<"; "<<endl;
        out<<"yRobot = "<<yRobot<<"; "<<endl;
        out<<"robot = plot(xRobot, yRobot, 'bo','MarkerFaceColor','b');"<<endl;

        //stampa posizione laser
//        double xLaser=map.lastRobotPose(Config::robotID)->x()+0.2085*cos(map.lastRobotPose(Config::robotID)->theta()+M_PI_2);
//        double yLaser=map.lastRobotPose(Config::robotID)->y()+0.2085*sin(map.lastRobotPose(Config::robotID)->theta()+M_PI_2);

//        out<<"xLaser = "<<xLaser<<"; "<<endl;
//        out<<"yLaser = "<<yLaser<<"; "<<endl;
//        out<<"plot(xLaser, yLaser, 'r+','MarkerFaceColor','r');"<<endl;


        //legenda
        if(Config::PRM::pointNumberFrontier==0){
            out<<"legend([walls(1); frontiers(1); edges(1); nodes(1); path; robot],'Wall','Frontier','Edge','Node','Path','Robot','location','northeastoutside');"<<endl;
        }
        else{
            out<<"legend([walls(1); frontiers(1); edges(1); nodes(1); frontiersNodes(1); path; robot],'Wall','Frontier','Edge','Node','Frontier Node','Path','Robot','location','northeastoutside');"<<endl;
        }
        //salva immagine
        QString imagename=pathFolder;
        imagename.append(QString("_map_%1.png").arg(seconds));

        QDateTime now = QDateTime::currentDateTime();
        out<<"dateString = '" << now.toString("dd MMMM yyyy, hh:mm:ss") <<"';" << endl;
        out<<"title(dateString);"<<endl;

        out<<"print(f,'-dpng','"<<imagename<<"');"<<endl;
        out<<"close all;";


        QString filename2= dataPath;
        filename2.append(QString("/data_%1.m").arg(seconds));
        QFile* output2= new QFile(filename2);
        if(!output2->open(QIODevice::WriteOnly | QIODevice::Text)){
            qDebug()<<"ERRORE. Impossibile creare il file!!!";
        }
        else{
            QTextStream out2(output2);
            out2<<"xWalls=["<<xWallsIni<<"; "<< xWallsFin <<"];"<<endl;
            out2<<"yWalls=["<<yWallsIni<<"; "<< yWallsFin <<"];"<<endl;
            out2<<"xFrontiers=["<<xFrontiersIni<<"; "<< xFrontiersFin <<"];"<<endl;
            out2<<"yFrontiers=["<<yFrontiersIni<<"; "<< yFrontiersFin <<"];"<<endl;
            out2<<"xRobot = "<<xRobot<<"; "<<endl;
            out2<<"yRobot = "<<yRobot<<"; "<<endl;
            out2<<"seconds = "<<seconds<<"; "<<endl;
            out2<<"xPath =["<<xPath<<"];"<<endl;
            out2<<"yPath =["<<yPath<<"];"<<endl;
        }
        output2->close();
    }
    output->close();

}

void PRMAlgorithm::handleNewMap(Map map){
    updatePRM(map);
}

void PRMAlgorithm::run(){
    exec();
}


QList<PRMPath> PRMAlgorithm::getPaths(Point* destination){
    stateMutex->lock();
    ldbg<<"PRMAlgorithm: Finding paths to: "<<*destination<<endl;
    QElapsedTimer timer;
    timer.start();
    QList<PRMPath> pathList = QList<PRMPath>();
    if(iterationNumber==0){
        ldbg<<"PRMAlgorithm: Invalid map"<<endl;
        return pathList;
    }
    if(aStar == NULL){
        ldbg<<"PRM A*: New A*"<<endl;
        aStar=new AStarAlgorithmPRM(oldMap, graph);
    }
    for(int alpha=1; alpha<=Config::PRM::pathNumber;alpha++){
        PRMPath path = aStar->getPath(alpha, destination);
        if(!path.isEmpty()){
            pathList.append(path);
        }
    }
    if(!pathList.empty()){
//        plotPaths(pathList, frontierID);
    }
    frontierID=frontierID+1;
    ldbg<<"PRM A*: Found "<<pathList.size()<<" paths in "<<timer.elapsed()<<"ms"<<endl<<endl;
    stateMutex->unlock();
    return pathList;
}

void PRMAlgorithm::plotPaths(QList<PRMPath> paths, int frontierID){
    QString filename=fullPath;
    int seconds=startTime.elapsed()/1000;
    filename.append(QString("/script_%1_path_%2.m").arg(seconds).arg(frontierID));
    QFile* output= new QFile(filename);
    if(!output->open(QIODevice::WriteOnly | QIODevice::Text)){
        qDebug()<<"ERRORE. Impossibile creare il file!!!";
    }
    else{
        QTextStream out(output);
        out<<"close all; clear all; clc;"<<endl;
        out<<"%graphics_toolkit('gnuplot');"<<endl;
        out<<"f=figure(1);"<<endl;
        out<<"hold on;"<<endl;

        //stampa muri
        QString xWallsIni= "";
        QString yWallsIni= "";
        QString xWallsFin= "";
        QString yWallsFin= "";
        foreach(LineSegment* w, oldMap.walls()){
            xWallsIni.append(" ").append(QString::number(w->x1(),'f',4));
            yWallsIni.append(" ").append(QString::number(w->y1(),'f',4));
            xWallsFin.append(" ").append(QString::number(w->x2(),'f',4));
            yWallsFin.append(" ").append(QString::number(w->y2(),'f',4));
        }
        out<<"xWalls=["<<xWallsIni<<"; "<< xWallsFin <<"];"<<endl;
        out<<"yWalls=["<<yWallsIni<<"; "<< yWallsFin <<"];"<<endl;
        out<<"walls = plot(xWalls, yWalls, 'k');"<<endl;

        //stampa frontiere
        QString xFrontiersIni= "";
        QString yFrontiersIni= "";
        QString xFrontiersFin= "";
        QString yFrontiersFin= "";
        foreach(LineSegment* f, oldMap.frontiers()){
            xFrontiersIni.append(" ").append(QString::number(f->x1(),'f',4));
            yFrontiersIni.append(" ").append(QString::number(f->y1(),'f',4));
            xFrontiersFin.append(" ").append(QString::number(f->x2(),'f',4));
            yFrontiersFin.append(" ").append(QString::number(f->y2(),'f',4));
        }
        out<<"xFrontiers=["<<xFrontiersIni<<"; "<< xFrontiersFin <<"];"<<endl;
        out<<"yFrontiers=["<<yFrontiersIni<<"; "<< yFrontiersFin <<"];"<<endl;
        out<<"frontiers = plot(xFrontiers, yFrontiers, '-.r');"<<endl;

        //stampa grafo
        graph.plot(&out);

        //dimensione plot
        int maxX = oldMap.getMaxX();
        int maxY = oldMap.getMaxY();
        int minX = oldMap.getMinX();
        int minY = oldMap.getMinY();
        out<<"axis(["<<minX-1<<" "<<maxX+1<<" "<<minY-1<<" "<<maxY+1<<"]);"<<endl;

        //stampa paths
        for(int i=0; i<paths.size(); i++){
            PRMPath path = paths.at(i);
            QString xPath = "", yPath= "";
            foreach(Point* p, path){
                xPath.append(" ").append(QString::number(p->x(),'f',4));
                yPath.append(" ").append(QString::number(p->y(),'f',4));
            }
            out<<"xPath"<<i<<"=["<<xPath<<"];"<<endl;
            out<<"yPath"<<i<<"=["<<yPath<<"];"<<endl;
            out<<"path"<<i<<"=plot(xPath"<<i<<", yPath"<<i<<","<<color.at(i%color.size())<<");"<<endl;
        }

        //stampa della destinazione
        Point* destination=paths.last().last();
        out<<"destination = plot("<<(QString::number(destination->x(),'f',4))<<", "<<(QString::number(destination->y(),'f',4))<<", 'mo', 'MarkerFaceColor', 'm');"<<endl;


        //stampa posizione robot
        QString xRobot = "", yRobot = "";
        xRobot.append(QString::number(oldMap.lastRobotPose(Config::robotID)->x(),'f',4));
        yRobot.append(QString::number(oldMap.lastRobotPose(Config::robotID)->y(),'f',4));
        out<<"robot = plot("<<xRobot<<", "<<yRobot<<", 'bo','MarkerFaceColor','b');"<<endl;

        //legenda
        if(Config::PRM::pointNumberFrontier==0){
            out<<"legend([walls(1); frontiers(1); edges(1); nodes(1); destination; robot],'Wall','Frontier','Edge','Node','Destination','Robot','location','northeastoutside');"<<endl;
        }
        else{
            out<<"legend([walls(1); frontiers(1); edges(1); nodes(1); frontiersNodes(1); destination; robot],'Wall','Frontier','Edge','Node','Frontier Node','Destination','Robot','location','northeastoutside');"<<endl;
        }
        //salva immagine

        QDateTime now = QDateTime::currentDateTime();
        out<<"dateString = '" << now.toString("dd MMMM yyyy, hh:mm:ss") <<"';" << endl;
        out<<"title(dateString);"<<endl;


        QString imagename=pathFolder;
        imagename.append(QString("_path_%1_%2.png").arg(seconds).arg(frontierID));
        out<<"print(f,'-dpng','"<<imagename<<"');"<<endl;
        out<<"close all;";
    }
    output->close();
}

bool PRMAlgorithm::comapareDouble(double a, double b){
    const double epsilon = 0.0001;
    return (qAbs(a-b)<= epsilon);
}

Data::Pose PRMAlgorithm::getOldRobotPose(){
    return Data::Pose(xRobot,yRobot,0);
}

void PRMAlgorithm::setCurrentPath(PRMPath path){
    currentPath=path;
}


/*SLOT Called by this->sigStartTimerEM*/
void PRMAlgorithm::onStartTimerPP()
{
    ldbg<<"Starting timer"<<endl;
    plotTimer->start(PLOT_TIMEOUT);
}

/*SLOT Called by this->sigStopTimer*/
void PRMAlgorithm::onStopTimer(){
    plotTimer->stop();
}

void PRMAlgorithm::onTimeout(){
    plot(oldMap);

}
}

