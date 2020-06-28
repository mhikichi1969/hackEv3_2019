#ifndef __BLOCK_BINGO_H__
#define __BLOCK_BINGO_H__

#include "Area.h"
#include "Distance.h"
#include "Runner.h"
#include "CarryBlock.h"
#include "HBTtask.h"
#include "Timer.h"
#include "HPolling.h"

class BlockBingo
{
    public:
        BlockBingo(HBTtask *bt,Timer *time,int course);
        int selectCarry();
        void getRoute(int node,int route[]);
        int carryBlock(int st_node);
        int getGoalCircleId();
        int currentNode();
        bool finishCheck();
        int toExit();
        void decodeBt();
        COLOR getBlockColor(int node);
        void turnRunnerAfterThrow(int afterdir);
        COLOR guessColor(int nodeid,hsv_t hsv);
        double getHueDistance(double ang1,double ang2);
        int getRestoredBlockMode();

    private:
        HBTtask *mBt;
        Area *mArea;
        Distance *mDistance;
        Runner *mRunner;
       // Runner *tmpRunner=nullptr;
        int tmpNodeid;
        DIR tmpDir;
        CarryBlock *mCarryBlock;
        Timer *mTimer;
        bool finish=false;

        int goal_num;

        int get_restored_block;
};


#endif
