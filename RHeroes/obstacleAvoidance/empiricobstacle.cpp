#include "empiricobstacle.h"
#include "abstractobstaclemanager.h"
#include <data/action.h>

#define THRESHOLD 0.23
#define RUOTASINISTRA_MIN 10
#define RUOTASINISTRA_MED 20
#define RUOTASINISTRA_MAX 40
#define RUOTADESTRA_MIN -10
#define RUOTADESTRA_MED -20
#define RUOTADESTRA_MAX -40

enum reactiveBehaviorEnum {DEACTIVATED,FIRSTTIME,EXEC};
enum typeMovementEnum {LLLFRRR,LLLFR,LLLF,LLL,LL,LFRRR,LFR,LF,L,FRRR,FR,F,RRR,RR,R,S};
enum movementStateEnum {FRONT,RIGHT,LEFT,BACK};
movementStateEnum actualMovement ;
int typeMovement = S;

int reactiveFrontBehaviorStatus;
int reactiveBackBehaviorStatus;



EmpiricObstacle::EmpiricObstacle(bool isActive)
{
    status = isActive;
}

void EmpiricObstacle::setStatus(bool isActive)
{
    status = isActive;
    reactiveBackBehaviorStatus = DEACTIVATED;
    reactiveFrontBehaviorStatus = DEACTIVATED;
}

void EmpiricObstacle::handleFrontSonarData(const Data::SonarData &sonar, QQueue<Data::Action *> normalActionQueue)
{    ldbg << "Robot Controller: Using front "<< sonar.getMinSonarName() << ". He say: " << sonar.getMinDistance() << endl;
    ldbg<<sonar.getMinSonarName()<<endl;

    if (sonar.getMinDistance()<THRESHOLD)
        handleFrontObstacle(sonar);
    else
    {
        ldbg<<"Robot Controller: Reactive Behavior Status "<<reactiveFrontBehaviorStatus<<endl;
        ldbg<<normalActionQueue->size()<<endl;


        if (reactiveFrontBehaviorStatus > DEACTIVATED && normalActionQueue->size()==0)
        {
            slowMotion = false;
            reactiveFrontBehaviorStatus = DEACTIVATED;
            if(haveReceivedWaypoint)
                tryReachWaypoint();
            else
                tryRefindPathFrontier();
        }
    }
}


void EmpiricObstacle::changeReactiveFSM(int reactiveBehaviorStatus)
{
    if (reactiveBehaviorStatus == EXEC || reactiveBehaviorStatus == FIRSTTIME)
    {
        ldbg << "RobotController: Reactive Behavior Status Deactivated."<<endl;
        reactiveBehaviorStatus = DEACTIVATED;
    }

    if (reactiveBehaviorStatus == DEACTIVATED)
    {
        stopRobot(true);
        ldbg << "RobotController: Reset Reactive Behaviour Status."<<endl;
        reactiveBehaviorStatus = FIRSTTIME;
    }
    else if (reactiveBehaviorStatus == FIRSTTIME)
    {
        ldbg<<"RobotController: Reactive Behaviour Status Executed."<<endl;
        reactiveBehaviorStatus = EXEC;
    }
}

void EmpiricObstacle::handleFrontObstacle(const Data::SonarData &sonar)
{
    double distanceLeftL = sonar.getFront(0);
    double distanceLeft = sonar.getFront(1);
    double distanceFront = sonar.getFront(2);
    double distanceRight = sonar.getFront(3);
    double distanceRightR = sonar.getFront(4);

    ldbg << "Robot Controller: FRR vale " << distanceRightR<<endl;
    ldbg << "Robot Controller: FR vale " << distanceRight<<endl;
    ldbg << "Robot Controller: F vale " << distanceFront<<endl;
    ldbg << "Robot Controller: FL vale " << distanceLeft<<endl;
    ldbg << "Robot Controller: FLL vale " << distanceLeftL<<endl;

    struct tm* timeinfo = getActualTime();

    ldbg<<"Robot Controller: Current local time and date: "<< asctime(timeinfo)<<endl;

    changeReactiveFSM(reactiveFrontBehaviorStatus);

    Pose actualPose = actualState->getPose();
    ldbg << "Robot Controller: Actual Pose ( " << actualPose.getX() << " , " << actualPose.getY() << " , " <<fromRadiantToDegree(actualPose.getTheta())<<endl;
    ldbg<< "Robot Controller: reactiveBehaviorStatus "<<reactiveBehaviorStatus<<endl;

    if (reactiveBehaviorStatus > DEACTIVATED)
    {
        ldbg<<"Robot Controller: First time, normal action queue size "<<normalActionQueue->size()<<endl;
        ldbg<<"Typemovement = "<<typeMovement<<endl;
        lastFrontSonarData = sonar;

        if (!isKenaf)
            slowMotion = true;
        obstacleAvoidanceEmpiricHandler(distanceRightR, distanceLeftL, distanceLeft, distanceFront, distanceRight);
    }

    useHybridControl = false;
}

void EmpiricObstacle::tryReachWaypoint()
{
    if (wayPointCounter<10)
    {
        ldbg<<"Robot Controller: I have a waypoint. I'm trying " <<wayPointCounter <<endl;
        wayPointCounter++;
        handleWaypoint(actualWaypoint);
    }
    else
    {
        ldbg<<"Robot Controller: I can't reach the waypoint."<<endl;
        delete actualWaypoint;
        sendSonarMessage();
        haveReceivedWaypoint = false;
        stopRobot(true);
    }
}

void EmpiricObstacle::tryRefindPathFrontier()
{
    {
        ldbg<<"Robot Controller: OldFrontier is ( "<<oldFrontier->x()<<" , "<<oldFrontier->y() << endl;
        if (refindPathCounter<20)
        {
            refindPathCounter++;
            ldbg<<"Robot Controller: refind path"<<endl;
            ldbg<<"Robot Controller: actualFrontier is( "<<actualFrontier->x()<<" , "<<actualFrontier->y() << endl;
            emit sigRestartExplorationRCM(actualFrontier->x(), actualFrontier->y());
        }
        else
        {
            ldbg<<"Robot Controller: restart exploration."<<endl;
            onRestartExploration();
        }

    }
}

void EmpiricObstacle::obstacleAvoidanceEmpiricHandler(double distanceRightR, double distanceLeft, double distanceRight, double distanceFront, double distanceLeftL)
{
    if (distanceLeftL < THRESHOLD)
    {
        if (distanceLeft < THRESHOLD)
        {
            if (distanceFront < THRESHOLD)
            {
                if (distanceRight < THRESHOLD)
                {
                    if (distanceRightR < THRESHOLD)
                    {
                        if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLLFRRR)
                        {
                            ldbg << "Caso LL_L_F_R_RR: Ostacoli ovunque"<<endl;
                            moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                            typeMovement = LLLFRRR;
                            ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLFRRR)
                        {
                            ldbg << "Typemovement was LLLFRRR. Now is" << typeMovement <<endl;
                            reactiveBehaviorStatus = DEACTIVATED;
                        }
                    }
                    else
                    {
                        if (reactiveBehaviorStatus == FIRSTTIME)
                        {
                            ldbg << "Caso LL_L_F_R: Estrema destra libera"<<endl;
                            moveRobot(0,VAI_INDIETRO,0);
                            moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                            typeMovement = LLLFR;
                            ldbg<< typeMovement <<endl;
                        }
                        else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLFR)
                        {
                            // //ldbg << "Typemovement was LLLFR. Now is" << typeMovement <<endl;
                            reactiveBehaviorStatus = DEACTIVATED;
                        }
                    }
                }
                else
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLLF)
                    {

                        ldbg << "Caso LL_L_F: Destra libera"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LLLF;
                        ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLLF)
                    {
                        ldbg << "Typemovement was LLLF. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LLL)
                {

                    ldbg << "Caso LL_L: Sinistra occupata"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                    typeMovement = LLL;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=LLL)
                {
                    ldbg << "Typemovement was LLL. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LL)
            {

                ldbg << "Caso LL: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = LL;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=LL)
            {
                ldbg << "Typemovement was LL. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceLeft < THRESHOLD)
    {
        if (distanceFront < THRESHOLD)
        {
            if (distanceRight < THRESHOLD)
            {
                if (distanceRightR < THRESHOLD)
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LFRRR)
                    {

                        ldbg << "Caso L_F_R_RR: Possibile pertugio estrema sinistra"<<endl;
                        moveRobot(0,VAI_INDIETRO,0);
                        moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                        typeMovement = LFRRR;
                        ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LFRRR)
                    {
                        ldbg << "Typemovement was LFRR. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
                else
                {
                    if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LFR)
                    {

                        ldbg << "Caso L_F_R: Estremi liberi. Vado indietro"<<endl;
                        moveRobot(0,VAI_INDIETRO,RUOTADESTRA_MAX);
                        typeMovement = LFR;
                        ldbg<< typeMovement <<endl;
                    }
                    else if(reactiveBehaviorStatus == EXEC && typeMovement!=LFR)
                    {
                        ldbg << "Typemovement was LFR. Now is" << typeMovement <<endl;
                        reactiveBehaviorStatus = DEACTIVATED;
                    }
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=LF)
                {

                    ldbg << "Caso L_F: Destra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTADESTRA_MAX,VAI_AVANTI,0);
                    typeMovement = LF;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=LF)
                {
                    ldbg << "Typemovement was LF. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)
            {

                ldbg << "Caso L: Sinistra occupata"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTADESTRA_MED,VAI_AVANTI,0);
                typeMovement = L;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=L)
            {
                ldbg << "Typemovement was L. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }

        }
    }
    else if (distanceFront < THRESHOLD)
    {
        if (distanceRight < THRESHOLD)
        {
            if (distanceRightR < THRESHOLD)
            {
                if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=FRRR)
                {

                    ldbg << "Caso F_R_RR: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FRRR;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=FRRR)
                {
                    ////ldbg << "Typemovement was FRR. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
            else
            {
                if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=FR)
                {

                    ldbg << "Caso F_R: Sinistra libera"<<endl;
                    moveRobot(0,VAI_INDIETRO,0);
                    moveRobot(RUOTASINISTRA_MAX,VAI_AVANTI,0);
                    typeMovement = FR;
                    ldbg<< typeMovement <<endl;
                }
                else if(reactiveBehaviorStatus == EXEC && typeMovement!=FR)
                {
                    ldbg << "Typemovement was FR. Now is" << typeMovement <<endl;
                    reactiveBehaviorStatus = DEACTIVATED;
                }
            }
        }
        else
        {

            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=F)
            {

                ldbg << "Caso F: Caso indecisione. Vado indietro."<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                typeMovement = F;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=F)
            {
                ldbg << "Typemovement was F. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
    }
    else if (distanceRight < THRESHOLD)
    {
        if (distanceRightR < THRESHOLD)
        {
            if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=RRR)
            {
                ldbg << "Caso R_RR: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = RRR;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC && typeMovement!=RRR)
            {
                ldbg << "Typemovement was RRR. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;
            }
        }
        else
        {
            if (reactiveBehaviorStatus == FIRSTTIME)// && typeMovement!=R)
            {

                ldbg << "Caso R: Sinistra libera"<<endl;
                moveRobot(0,VAI_INDIETRO,0);
                moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
                typeMovement = R;
                ldbg<< typeMovement <<endl;
            }
            else if(reactiveBehaviorStatus == EXEC  && typeMovement!=R)
            {
                ldbg << "Typemovement was R. Now is" << typeMovement <<endl;
                reactiveBehaviorStatus = DEACTIVATED;;
            }
        }
    }
    else if (distanceRightR < THRESHOLD)
    {
        if (reactiveBehaviorStatus == FIRSTTIME)//&& typeMovement!=RR)
        {

            ldbg << "Caso_RR: Sinistra libera"<<endl;
            moveRobot(0,VAI_INDIETRO,0);
            moveRobot(RUOTASINISTRA_MED,VAI_AVANTI,0);
            typeMovement = RR;
            ldbg<< typeMovement <<endl;
        }
        else if(reactiveBehaviorStatus == EXEC && typeMovement!=RR)
        {
            ////ldbg << "Typemovement was RR. Now is" << typeMovement <<endl;
            reactiveBehaviorStatus = DEACTIVATED;
        }
    }
    else
        ldbg<<"Waiting movement typeMovement" << typeMovement << endl;
}

void EmpiricObstacle::handleBackObstacle(const Data::SonarData &sonar);
{

    double distanceLeftL = sonar.getBack(0);
    double distanceLeft = sonar.getBack(1);
    double distanceFront = sonar.getBack(2);
    double distanceRight = sonar.getBack(3);
    double distanceRightR = sonar.getBack(4);

    ldbg << "Robot Controller: BRR vale " << distanceRightR<<endl;
    ldbg << "Robot Controller: BR vale " << distanceRight<<endl;
    ldbg << "Robot Controller: B " << distanceFront<<endl;
    ldbg << "Robot Controller: BL " << distanceLeft<<endl;
    ldbg << "Robot Controller: BLL " << distanceLeftL<<endl;

    struct tm* timeinfo = getActualTime();

    ldbg<<"Robot Controller: Current local time and date: "<< asctime(timeinfo)<<endl;

    changeReactiveFSM(reactiveBackBehaviorStatus);


    Pose actualPose = actualState->getPose();
    ldbg << "Robot Controller: Actual Pose ( " << actualPose.getX() << " , " << actualPose.getY() << " , " <<fromRadiantToDegree(actualPose.getTheta())<<endl;
    ldbg<< "Robot Controller: reactiveBehavior"<<reactiveBackBehaviorStatus<<endl;


    if (reactiveBackBehaviorStatus == FIRSTTIME)
    {
        moveRobot(0,VAI_AVANTI,0);
    }
}

void EmpiricObstacle::handleBackSonarData(const Data::SonarData &sonar, QQueue<Data::Action *> normalActionQueue)
{
    QString sonarName = sonar.getMinSonarName();
    ldbg << "Robot Controller: Using back "<< sonarName << ". He say: " << sonar.getMinBackDistance() << endl;
    if(sonar.getMinBackDistance()<THRESHOLD)
        handleBackObstacle(sonar);
    else if (reactiveBehaviorStatus > DEACTIVATED && normalActionQueue->size()==0)
    {
        slowMotion = false;
        reactiveBehaviorStatus = DEACTIVATED;
        if(haveReceivedWaypoint)
            tryReachWaypoint();
        else
            tryRefindPathFrontier();
    }
}


