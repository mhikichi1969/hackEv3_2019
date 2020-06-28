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
    //static char buf[256];
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


// 運搬前の黒ブロック（ボーナスに無い黒ブロック）の周りに、ブロックサークルと運搬対象ブロックとは別の同色のブロックが存在しているかチェック
bool CarryBlock::checkBlockAroundBlackBlock(int nodeid, COLOR col)
{
    int idx = seachBlackBlock();
    if(idx==-1)
            return false; // 黒ブロック移動済みならそのまま移動対象
    BlockCircle *bp = (BlockCircle*)mArea->getBlockPlace(idx);  // ブロックサークル取得

    COLOR block_circle_color = bp->getColor(); // ブロックサークル色
    if(block_circle_color!=col) // 違う色ならそのまま移動対象
        return false;

    BlockPlace **cross_bp = bp->getSlashPlaces();
    for (int i=0;i<4;i++) {
        int cross_id = cross_bp[i]->getNodeid();
        if(cross_id==nodeid)
            return false; // 移動対象が黒ブロックの周りであれば、そのまま移動対象
    }
    bool find_block=false; // 同色のブロックがあるかどうか
    for (int i=0;i<4;i++) {
        Block *bk = cross_bp[i]->getBlock();
        if(bk!=nullptr) {
            COLOR bk_color = bk->getColor();
            if(block_circle_color==bk_color) {
                find_block=true;
                break;
            }
        }
    }

    return find_block;
}

// ボーナス以外のブロックサークルに黒ブロックがあればそのidを返す。
int CarryBlock::seachBlackBlock()
{
    int idx;
    for(int i=0;i<8;i++) {
        idx = block_circle_idx[i];
        BlockCircle *bp = (BlockCircle*)mArea->getBlockPlace(idx);  // ブロックサークル取得
        Block *bk = bp->getBlock();
        if(i+1!=mArea->getBonusNo() && bk!=nullptr && bk->getColor()==COLOR::BLACK)  // ボーナス以外のブロックサークルに黒ブロックがあるか？
            return idx;
    }
    return -1;
}