#ifndef __BLOCK_PLACE__
#define __BLOCK_PLACE__

#include "Block.h"
#include "BingoEnum.h"

class BlockPlace
{
    public:
        BlockPlace(int node);
        void resetBlock();
        int getNodeid();
        void getNodeXY(int pt[]);

        void addBlock(Block *block);
        void delBlock();
        Block *getBlock();
        BPKIND getKind();

        void setNextPlace(BlockPlace *up,BlockPlace *right,BlockPlace *down,BlockPlace *left);
        void setSlashPlace(BlockPlace *upright,BlockPlace *downright,BlockPlace *downleft,BlockPlace *upleft);

        BlockPlace** getSlashPlaces();
        int getCountOfBlock();
        void getBlocks(Block *bk[]);

    protected:
        BPKIND kind;

    private:
        int nodeid;
        Block *block[10];
        int block_idx;

        BlockPlace *next[4];
        BlockPlace *slash[4];

};


#endif