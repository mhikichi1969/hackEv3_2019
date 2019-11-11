#include "Timer.h"
#include "util.h"
#include "Const.h"

Timer::Timer(int course):
    mCourse(course)
{
    goalNode = (course==0)?34:14; //出口の番号
    goalDir = (course==0)?DIR::E : DIR::W;  //出口の方向
    parkingTime = (course==0)?L_GARAGE : R_GARAGE; // 駐車へ向かう時間

    mClock = new Clock();
    allTime = TIMEOUT;
    
}

void Timer::setDistance(Distance *dis)
{
    mDistance = dis;
}

void Timer::setStart()
{
    startTime = mClock->now();
}

void Timer::calcRemainTime()
{
    remainTime = allTime - (mClock->now() - startTime);
}

double Timer::getToParkingCost(int from)
{
    mDistance->setCheckBlock(false);
    mDistance->setStart(from);
    mDistance->calcDistance();
    mDistance->calcTurncostAll(goalDir);

    //mDistance->debugPrint();

    return mDistance->getLenAndTurnCost(goalNode);
}

bool Timer::carryJudge(int from,double carry_cost)
{

    calcRemainTime();
    double parkingCost = getToParkingCost(from);

    char buf[256];
    sprintf(buf,"TO rt%d,c:%2.1f,p:%2.1f,%d",remainTime,carry_cost,parkingCost,from);
    msg_f(buf,4);

    return remainTime/1000.0>carry_cost+4.5+parkingCost+parkingTime/1000.0;
}

void Timer::getRoute(int from, int* route)
{
    mDistance->getRoute(from, goalNode, route);
}

int Timer::getGoalNode()
{
    char buf[256];
    sprintf(buf,"timeout goal %d",goalNode);
    msg_f(buf,10);
    return goalNode;
}

