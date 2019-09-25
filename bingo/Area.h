#ifndef __AREA_H__
#define __AREA_H__

#include "BlockPlace.h"
#include "Block.h"
#include "Runner.h"

class Area
{
    public:
         Area(int idx);
        BlockPlace *getBlockPlace(int idx);
        void getBlockList(int node[]);
        void getExitList(int exit[]);
        Runner* getRunner();
        Block* getBlock(int id);
        void setBlock(int id,Block *bk);
        void setBonusNo(int no);
        int getBonusNo();
        int remainBlock();
        void setDefaultBlock();
        void modifyBlockColor(int no,COLOR col );
        void modifyBlockColorByNodeId(int nodeid,COLOR col);
        void cleanBlocks();
        void assignBlockList();
        int *getAssingPointer();
    private:
        int course;
        int bonus;
        int exit[2];
        BlockPlace *bp[49];
        Runner *runner;

        int assign[5];



};

#endif