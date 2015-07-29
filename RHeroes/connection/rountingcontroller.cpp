#include "rountingcontroller.h"
#include "shared/utilities.h"
#include "shared/config.h"

#include <QTimer>
#include <QDebug>
#include <QDateTime>
#define HOLDDOWN_TIMEOUT_MSEC   5000

using namespace Data;
namespace Connection{

bool activateRouting = true;

RoutingController::RoutingController(QObject *parent, uint nRobot) :
    QObject(parent), nrobot(nRobot), routingTable(QHash<QString, t_connection>())
{
    if(DISTANCE_VECTOR_REQUEST_TIMER > 0)
    {
        QTimer *timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(updateDistanceVector()));
        timer->start(DISTANCE_VECTOR_REQUEST_TIMER);
    }

    distanceVectorTimer= new QTimer(this);
}

void RoutingController::updateDirectConnection(QString peer, double signalStrength)
{
    if(peer == robotNameFromIndex(nrobot))
        return;
    bool modified = false;
    bool added = false;
    t_connection old;
    t_connection predefined = {RP_UNDEFINED, UNDEFINED_COST};
    if(signalStrength > -90){
        //qDebug("in %f",signalStrength);
		ldbg << "Si è connesso " << peer.toAscii().data() << " con forza " << signalStrength << endl;
        //if(signalStrength > -300+NAN_SIGNAL_STRENGTH){
			if(routingTable.contains(peer))
			{
				old = routingTable.value(peer);
				if(old.nextHop != peer)//Before we didn't have direct connection, but now it is direct
				{
					modified = true;

				}
			}
			else
			{
                modified = true;
				added = true;
			}
            if(modified){
			t_connection conn = {peer, 1};
			routingTable.insert(peer,conn);
            qDebug() << "M1: Inserted " << peer.toAscii().data() << " via " << conn.nextHop.toAscii().data() << " with cost " << conn.cost;

				//ldbg << "direct connecttion modified with " << peer << "signal Strength is: " << signalStrength << endl;
				emit signalNewConnectedPeer(peer);
			}
			if(added){
			   // ldbg << "direct connection stablished with " << peer << "signal Strength is: " << signalStrength << endl;
                emit signalNewReachablePeer(peer);
			}
    //}
    }
    else {
        old = routingTable.value(peer, predefined);
        if(old.nextHop == RP_UNDEFINED) {
           // ldbg << "there wasn't connection ... signal is lower than -90 !!!" << endl;
            return; //there wasn't connection... nothing changes!
        }
        else{
            if(old.nextHop == peer){ // There was a direct connection... now broken!
                modified = true;
                routingTable.remove(peer);
                qDebug() << "M2: Removed " << peer.toAscii().data();
             //   ldbg << "Disconnected with " << peer << endl;
                foreach(QString inDirectPeer , routingTable.keys()) { // remove other peers related to removed peer, by Sajjad
                    if(routingTable[inDirectPeer].nextHop == peer) {
                        routingTable.remove(inDirectPeer);
                        qDebug() << "M3: Removed" << inDirectPeer.toAscii().data();
                    }
                }
            }
        }
    }
    if(modified){
        qDebug("Modified 1 sent!");
        //updateDistanceVector();
        distanceVectorTimer->stop();
        distanceVectorTimer->singleShot(2000,this,SLOT(updateDistanceVector()));
//        //DEBUG
//        ldbg << "Direct connections updated: " << endl;
//        ldbg << "dest \t nextHop" << endl;
//        foreach(QString p, routingTable.keys())
//        {
//            ldbg << p << "\t" << routingTable[p].nextHop << endl;
//        }//end debug
    }
}

void RoutingController::processReceivedDistanceVector(const QString &peer, const QHash<QString, int> &dv)
{
    if((!routingTable.contains(peer))||routingTable[peer].nextHop!=peer){
        return;
    }
    bool modified = false;
    bool added = false;
    //ldbg << "Recived Distance Vector From: " << peer << endl;
    /*foreach(QString dest, dv.keys()){
        ldbg << "dest: " << dest << " cost: " << dv[dest] << endl;
    }*/
    foreach(QString dest,routingTable.keys()){
        if((dest!=routingTable[dest].nextHop)&&(routingTable[dest].nextHop==peer)){
            if(!dv.contains(dest)){
                routingTable.remove(dest);
                /*HoldDown newHoldDown;
                newHoldDown.dest=dest;
                newHoldDown.nextHop=peer;
                newHoldDown.until=QDateTime::currentMSecsSinceEpoch()+HOLDDOWN_TIMEOUT_MSEC;
                holdDown.push_back(newHoldDown);*/
                qDebug() << "M4: Removed" << dest.toAscii().data();
                modified=true;
            }
            else if(routingTable[dest].cost<dv[dest]){
                routingTable[dest].cost=dv[dest];
                qDebug() << "M5: Updated cost of " << dest.toAscii().data() << " to " << dv[dest];
                modified=true;
            }
        }

    }

    foreach(QString dest, dv.keys()){
      //  ldbg << "destination: " << dest << endl;
        if(dest != robotNameFromIndex(nrobot) && dv[dest] < COUNT_TO_INFINITY_LIMIT){
            if(!routingTable.contains(dest)){
        //        ldbg << " was not in the routing table" << endl;
                //if(!checkHoldDown(dest,peer)){
                modified = true;
                added = true;
                t_connection conn = {peer, dv[dest]};                
                routingTable.insert(dest, conn);
                qDebug() << "M6 Inserted" << dest.toAscii().data() << "  via " << conn.nextHop.toAscii().data() << " with cost " << conn.cost;
                //}
            }else if(routingTable[dest].cost > dv[dest]){
          //      ldbg << " Better distance than " << routingTable[dest].nextHop << endl;
                //if(!checkHoldDown(dest,peer)){
                modified = true;
                t_connection conn = {peer, dv[dest]};
                routingTable.remove(dest);
                routingTable.insert(dest, conn);
                qDebug() << "M7 Updated" << dest.toAscii().data() << "  via " << conn.nextHop.toAscii().data() << " with cost " << conn.cost;
                //}
            }
            else {
            //    ldbg << " worse distance than " << routingTable[dest].nextHop << dv[dest] << ">" << routingTable[dest].cost << endl;
            }
        }

    }
    /*foreach(QString dest, dv.keys()){
        if (dest != robotNameFromIndex(nrobot) && routingTable[dest].nextHop.length() > 2 && routingTable[routingTable[dest].nextHop].nextHop.length() > 2 && routingTable[dest].nextHop == peer && routingTable[dest].nextHop != routingTable[routingTable[dest].nextHop].nextHop) {
           // ldbg << "changed dest of " << dest << " from " << routingTable[dest].nextHop << " with " << routingTable[routingTable[dest].nextHop].nextHop << endl;
            modified = true;
            t_connection conn = {routingTable[routingTable[dest].nextHop].nextHop, dv[dest]};
            routingTable.insert(dest, conn);
        }
    }*/
    if(modified){
        qDebug("Modified 2 sent!");
        //updateDistanceVector();
        distanceVectorTimer->stop();
        distanceVectorTimer->singleShot(5000,this,SLOT(updateDistanceVector()));
    }
    if(added)
        emit signalNewReachablePeer(peer);
}

QString RoutingController::getNextHopDestination(const QString &dest)
{
    if (!activateRouting)
    {
        return dest;
    }
    t_connection conn = {RP_UNDEFINED, UNDEFINED_COST};
    if(routingTable.contains(dest))
    {
        conn = routingTable.value(dest);
    }
    return conn.nextHop;
}

void RoutingController::updateDistanceVector()
{

    //qDebug("%s - Sending DV!",QTime::currentTime().toString().toAscii().data());
    QHash<QString /*destination*/, int /*cost*/> distanceVector;
    //ldbg << endl <<"Routing: Test Distance Vector | Peer | Cost" << endl << "--- | --- | ---" << endl;
    distanceVector.clear();
    int i = 0;
    foreach(QString peer, routingTable.keys())
    {
        distanceVector[peer] = routingTable[peer].cost + 1; /*done by sajjad*/
      //  ldbg << i++ << " | Peer " << peer << " | " << distanceVector[peer] << endl;
    }

    emit signalRoutingUpdated(distanceVector);
}

/*void RountingController::onUpdateHoldDown(){
    qint64 currentTime=QDateTime::currentMSecsSinceEpoch();

    while(holdDown.size()>0&&holdDown[0].until<currentTime){
        holdDown.pop_front();
    }
}

bool RountingController::checkHoldDown(const QString &dest, const QString &nextHop){
    foreach(HoldDown entry,holdDown){
        if((entry.dest==dest)&&(entry.nextHop==nextHop)){
            return true;
        }
    }
    return false;
}
*/
}
