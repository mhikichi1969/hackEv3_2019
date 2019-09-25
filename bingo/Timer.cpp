#include "Timer.h"
#include "util.h"

Timer::Timer(int course):
    mCourse(course)
{
    goalNode = (course==0)?34:14; //出口の番号
    goalDir = (course==0)?DIR::E : DIR::W;  //出口の方向
    parkingTime = (course==0)?4000 : 5000; // 駐車へ向かう時間

    mClock = new Clock();
    allTime = 120000;
    
    //allTime = 240000;

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

   // mDistance->debugPrint2();

    return mDistance->getLenAndTurnCost(goalNode);
}

bool Timer::carryJudge(int from,double carry_cost)
{

    calcRemainTime();
    double parkingCost = getToParkingCost(from);

    char buf[256];
   // sprintf(buf,"TO rt%d,cry%2.0f,pc%2.0f",remainTime,carry_cost,parkingCost);
   // msg_f(buf,0);

    return remainTime/1000.0>carry_cost+2.5+parkingCost;
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

