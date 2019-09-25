#include "CarryBlock.h"
#include "BlockCircle.h"

#include "util.h"

CarryBlock::CarryBlock(Area *area):
        mArea(area)
{
    

}

/* 運搬先候補をリストで返す①*/
void CarryBlock::getCarryList(COLOR color, int list[])
{
    //char buf[256];
    //sprintf(buf,"%d %d ",color,mArea->getBonusNo());
    // msg_f(buf,12);
    int cnt=0;
    for(int i=0;i<8;i++) {
        int idx = block_circle_idx[i];
        BlockCircle *bp = (BlockCircle*)mArea->getBlockPlace(idx);
        if ((bp->getBlock()==nullptr && bp->getColor()==color) || (color==COLOR::BLACK && mArea->getBonusNo()==bp->getid()) // 未投入で同一色か、黒でボーナスサークルなら候補
               // || (bp->getBlock()!=nullptr && bp->getBlock()->getColor()==COLOR::BLACK && mArea->getBonusNo()==bp->getid() &&  bp->getColor()==color )) {  // ボーナスサークルに黒が入っている
                || checkOnlyBlackBlock(bp) && mArea->getBonusNo()==bp->getid() &&  bp->getColor()==color) {  // ボーナスサークルに黒だけが入っている

            BlockPlace **cross = bp->getSlashPlaces();
            for(int k=0;k<4;k++) {
                if (cross[k]!=nullptr) {
                    list[cnt++] = cross[k]->getNodeid();
                }
            }
        }
    }
    list[cnt++] = 999;

}


bool CarryBlock::checkOnlyBlackBlock(BlockCircle *bc)
{
    Block *bk[10];
    //    msg_f("checkOnlyBlackBlock 1",0);

    bc->getBlocks(bk);
     //   msg_f("checkOnlyBlackBlock 2",0);

    int i=0;
    if(bk[0]==nullptr) return false;
    //msg_f("checkOnlyBlackBlock 3",0);

    while(bk[i]!=nullptr)
    {
      //  msg_f("checkOnlyBlackBlock 4",0);

        if(bk[i]->getColor()!=COLOR::BLACK)
            return false;
        i++;
    }

    return true;

}