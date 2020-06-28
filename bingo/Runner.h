#ifndef __RUNNER_H__
#define __RUNNER_H__

#include "BlockPlace.h"
#include "BingoEnum.h"
#include "Block.h"
#include "CrossCircle.h"
//#include "Area.h"

class Area;   //Areaと相互参照

class Runner
{
    public:
        Runner();
        void setStart(BlockPlace *bp);
        void setEnd(BlockPlace *bp);
        BlockPlace *getStart();
        BlockPlace *getEnd();


        void setDir(DIR dir);
        void carryBlock(Block *bk);
        Block *checkBlock();
        Block *releaseBlock();
        int turnRunner(int turn);
        int checkDirection(int from_[],int to[]);
        DIR getDir();
        Runner *makeClone();

        void setNext(int next);
        void setNext(BlockPlace *bp);
        void setArea(Area *a);
        int calcRoute();
        int calcDirection(int from_[],int to[]);
        COLOR getNextColor();
        int getNextNodeId();
        BlockPlace* getNextBlockPlace();
        bool isCircleBefore();
        void setCircleBefore(bool b);
        bool isOnLine();
       void setOnLine(bool b);

    private:
        DIR dir;

        BlockPlace *start;
        BlockPlace *end;

        Block *block;

        BlockPlace *mNext;
        Area *mArea;
        
        int sbp[2] = {0,0};
        int nbp[2] = {0,0};

        bool mCircleBefore;
        bool mOnLine;
};


#endif