#include "BlockPlace.h"
#include "util.h"

BlockPlace::BlockPlace(int node)
{
    kind=BPKIND::OTHER;
    nodeid = node;
    block_idx=-1;
    for(int i=0;i<10;i++)
        block[i]=nullptr;

}

void BlockPlace::resetBlock()
{
    block_idx=-1;
}

int BlockPlace::getNodeid()
{
    return nodeid;
}

void BlockPlace::getNodeXY(int pt[])
{
    //msg_f("BP:getXY",4);
    //char buf[256];
    //sprintf(buf,"BP:getXY : %d,%d,%d",nodeid, nodeid/7, nodeid%7);
    //msg_f(buf,6);

    pt[0]=nodeid/7;
    pt[1]=nodeid%7;
    //msg_f("BP:getXY:END",5);
}

void BlockPlace::addBlock(Block *bk)
{
    char buf[256];

    block[++block_idx] = bk;
}

void BlockPlace::delBlock()
{
    char buf[256];
    
     if (block_idx>=0) {
        block[block_idx--]=nullptr;
     }
}

Block* BlockPlace::getBlock()
{

    if (block_idx==-1)
        return nullptr;
    return block[block_idx];
}

void BlockPlace::getBlocks(Block *bk[])
{
    int cnt;
    for(cnt=0;cnt<block_idx+1;cnt++)
    {
        bk[cnt] = block[cnt];
    }
    bk[cnt] = nullptr;

}
int BlockPlace::getCountOfBlock()
{
    return block_idx+1;
}


BPKIND BlockPlace::getKind()
{
    return kind;
}

void BlockPlace::setNextPlace(BlockPlace *up,BlockPlace *right,BlockPlace *down,BlockPlace *left)
{
    next[0] = up;
    next[1] = right;
    next[2] = down;
    next[3] = left;

}
void BlockPlace::setSlashPlace(BlockPlace *upright,BlockPlace *downright,BlockPlace *downleft,BlockPlace *upleft)
{
    slash[0] = upright;
    slash[1] = downright;
    slash[2] = downleft;
    slash[3] = upleft;

}

BlockPlace** BlockPlace::getSlashPlaces()
{
    return slash;
}


