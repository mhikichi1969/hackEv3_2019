#ifndef __CARRY_BLOCK_H__
#define __CARRY_BLOCK_H__

#include "Area.h"
#include "BingoEnum.h"
#include "BlockCircle.h"

class CarryBlock
{
    public:
        CarryBlock(Area *area);
        void getCarryList(COLOR color,int list[]);
        bool checkOnlyBlackBlock(BlockCircle *bc);
        bool checkBlockAroundBlackBlock(int nodeid, COLOR col);
        int seachBlackBlock();



    private:
        Area *mArea;
        int block_circle_idx[8] = {8,10,12, 22, 26, 36,38,40};

};

#endif
