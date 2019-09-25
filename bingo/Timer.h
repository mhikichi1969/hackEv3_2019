#ifndef __TIMER_H__
#define __TIMER_H__
#include "Distance.h"
#include "BingoEnum.h"
#include "ev3api.h"

#include "Clock.h"

using namespace ev3api;

class Timer
{
    public:
        Timer(int course);
        void setDistance(Distance *dis);
        void setStart();
        void calcRemainTime();
        double getToParkingCost(int from);
        void getRoute(int from, int* route);
        bool carryJudge(int from,double carry_cost);
        int getGoalNode();

    private:
        Clock   *mClock;

        Distance *mDistance;
        int allTime;
        int startTime;
        int remainTime;
        int maxTime;
        int parkingTime;
        int mCourse;

        int goalNode;
        DIR goalDir;

};

#endif