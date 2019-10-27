#include "BlockBingo.h"
#include "util.h"
#include "BlockPlace.h"
#include "BlockCircle.h"
#include "BingoEnum.h"
#include "Block.h"
#include "Const.h"
#include "math.h"

BlockBingo::BlockBingo(HBTtask *bt, Timer *time,int course):
    mBt(bt)
{                                                                           
    mArea = new Area(course);
    mDistance  = new Distance(mArea);
    Distance *dis =  new Distance(mArea);
    mRunner = mArea->getRunner();
    mCarryBlock = new CarryBlock(mArea);
    mTimer = time;
    mTimer->setDistance(dis);

    get_restored_block=-1;
}

int BlockBingo::selectCarry()
{
    char buf[256];

    int node[20];
    mArea->getBlockList(node); // ブロックのある場所一覧
    //sprintf(buf,"Blk:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",node[0],node[1],node[2],node[3],node[4],node[5],node[6],node[7],node[8],node[9]);
   // msg_f(buf,0);
    Runner* dummy = mRunner->makeClone();
    BlockPlace *start = dummy->getStart();

    sprintf(buf,"start DIR %d",dummy->getDir());
    if(start->getNodeid()==44)
        msg_f(buf,0);
    
    int maxcost=0;
    int cost,cost2;

    int node_idx=0;

    int min_cost=1000;
    int min_cost_idx=-1;
    int origine_idx;
    bool move;
    int  toblock_endnode;
    move=false;


    if(start->getBlock()!=nullptr) {

        return start->getNodeid();
    }

    int st = start->getNodeid();
    for (node_idx=0;node[node_idx]!=-1;node_idx++) {
        dummy->setStart(mArea->getBlockPlace(node[node_idx]));
        mDistance->setStart(st);
        Block *tmp = mArea->getBlock(node[node_idx]); // 一時的にブロックを外す
        mDistance->calcDistance();
        mDistance->calcTurncostAll(dummy->getDir());

        if(node[node_idx]!=st)
            mDistance->setBlockinGoalCost(node[node_idx]);
        cost = mDistance->getLenAndTurnCost(node[node_idx]);    

       // mDistance->debugPrint2();
        
        mArea->setBlock(node[node_idx],tmp); // 一時的に外したブロックを元に戻す
        

        if (cost>99) continue; // ブロックまで行けない

        //黒ブロックのあるブロックサークル周りに同色のブロックがあれば対象外とする（ただし、運搬対象は除く）
       /* BlockPlace *bp_search = mArea->getBlockPlace(node[node_idx]);
        COLOR c = bp_search->getBlock()->getColor();
        if(mCarryBlock->checkBlockAroundBlackBlock(node[node_idx],c))
            continue;
            */


        // 運搬先読み
        toblock_endnode = node[node_idx];
        origine_idx=node_idx;
        if(mArea->getBlockPlace(node[node_idx])->getKind()==BPKIND::BLOCK) {
            sprintf(buf,"sel:%d %d",st,toblock_endnode);
           // msg_f(buf,12);

            DIR dir = mDistance->getGoalDirection(node[node_idx]);
            int newnode=node[node_idx];
            if(dir==DIR::N) 
                newnode-= 7;
            if(dir==DIR::S) 
                newnode += 7;
            if(dir==DIR::E) 
                newnode += 1;
            if(dir==DIR::W) 
                newnode -= 1;
            node[node_idx] = newnode;
            move=true;
        } 

        BlockPlace *bp = mArea->getBlockPlace(toblock_endnode);
        dummy->setStart(bp);
        Block *bk = bp->getBlock();
        dummy->carryBlock(bk);
        COLOR col = bk->getColor();
        int list[20];
        list[0]=999;
        if (col!=COLOR::NONE)
            mCarryBlock->getCarryList(col,list); //色が判明してる場合はそのリストを取得

        DIR runner_dir;
        if(toblock_endnode!=st)
            runner_dir = mDistance->getGoalDirection(toblock_endnode); // ブロックに入った時の方向
        else
            runner_dir = dummy->getDir();

       // dummy->setDir(runner_dir);

        mDistance->setStart(node[node_idx]);
        mDistance->calcDistance();
        mDistance->calcTurncostAll(runner_dir);
        if(move)  node[node_idx] = toblock_endnode;

       // mDistance->debugPrint();

        /* 複数の運搬先リストからコスト最小の場所を探す*/
        int mincost2=15; // ブロック色不明又はブロック退避の場合のペナルティ（要調整）
        int minidx=-1;
        for(int cnt=0;list[cnt]!=999;cnt++) {
            cost2 = mDistance->getLenAndTurnCost(list[cnt]);  
            if (mincost2>cost2) {
                mincost2=cost2;
                minidx=cnt;     
            } 
        } 

        if (min_cost>cost+mincost2) {
            min_cost=cost+mincost2;
            min_cost_idx=node_idx;     
        } 

    }

    delete dummy;
/*
    if (move && min_cost_idx==origine_idx) {
        sprintf(buf,"move %d",toblock_endnode);
        msg_f(buf,8);
        return toblock_endnode;
    }*/

  //  return node[min_cost_idx];

    //タイムアウト処理をする場合

    if(mTimer->carryJudge(st,min_cost)) {
         return node[min_cost_idx];
    } else {
        finish = true;
        return mTimer->getGoalNode();        
    }
}

int BlockBingo::carryBlock(int st_node)
{
    goal_num = -1;

    BlockPlace *bp = mArea->getBlockPlace(st_node); // スタート位置の置き
        
    Block *bk = mArea->getBlock(st_node); // ブロックを取り除く
    if(bk!=nullptr) {// ブロックがあった 
        mRunner->carryBlock(bk); // ブロックをつかむ
    }
    //Block *bk = mRunner->checkBlock(); // つかんでるブロック
        //Runner *dummy = mRunner->makeClone();
    COLOR col = bk->getColor(); // ブロックの色確認
    int list[20];
    mCarryBlock->getCarryList(col,list); //運搬対象対象の置き場取得

    char buf[256];
    sprintf(buf,"carry:%d",mRunner->getDir());
    sprintf(buf,"%d %d %d %d %d %d %d %d ",list[0],list[1],list[2],list[3],list[4],list[5],list[6],list[7] );
  //  if(st_node==4) msg_f(buf,9);
//
      //   DIR runner_dir = mDistance->getGoalDirection(st_node); // 開始位置の方向を予測
       // mRunner->setDir(runner_dir); 

        mDistance->setStart(st_node);
        mDistance->calcDistance();
        mDistance->calcTurncostAll(mRunner->getDir());
        /*if(bp->getKind()==BPKIND::BLOCK) {
            mDistance->calcTurncostAllPostProcess(bp->getNodeid(),mRunner->getDir());
            sprintf(buf,"carry:%d",mRunner->getDir());
          //  msg_f(buf,11);
        }*/

   

       // mDistance->debugPrint2();

        /* 複数の運搬先リストからコスト最小の場所を探す*/
        int mincost2=100;
        int minidx=-1;
        double cost2;
        for(int cnt=0;list[cnt]!=999;cnt++) {
            cost2 = mDistance->getLenAndTurnCost(list[cnt]);  
            if (mincost2>cost2) {
                mincost2=cost2;
                minidx=cnt;     
            } 
        } 
        // 移動場所がない場合は最も近い黒線上へ退避
        if(minidx==-1) {
            double mincost = 100;
            int minidx=-1;
            for(int idx=0;idx<49;idx++) {
                double cost = mDistance->getLenAndTurnCost(idx);  
                if(cost>0 && mincost>cost) {
                    mincost=cost;
                    minidx=idx;
                }
            }
           // mDistance->debugPrint2();
           // sprintf(buf,"taihi %d,%f",minidx,mincost);
           // msg_f(buf,9);

            return minidx;
        }


        //スローインの場所に到達
        Block *carryblock = mRunner->checkBlock();
        mRunner->setDir(mDistance->getGoalDirection(list[minidx]));  //9/21
        if(carryblock!=nullptr) {

            BlockPlace *bp = mArea->getBlockPlace(list[minidx]);

            BlockCircle **bc = (BlockCircle**)bp->getSlashPlaces();

            for(int i=0;i<4;i++) {
                /*if(bc[i]!=nullptr)
                    sprintf(buf,"%d %d %d",list[minidx],bc[i]->getColor(),carryblock->getColor());
                msg_f(buf,i+8);*/
                //if(bc[i]!=nullptr && bc[i]->getColor()==carryblock->getColor()) {
                if(bc[i]!=nullptr && (bc[i]->getColor()==carryblock->getColor() || 
                    carryblock->getColor() == COLOR::BLACK && bc[i]->getid() == mArea->getBonusNo() )) {
                    bc[i]->addBlock(carryblock);
                    mRunner->releaseBlock();
                    goal_num = bc[i]->getNodeid();
                    turnRunnerAfterThrow(i);
                    break;
                }
            }
        }

        return list[minidx];

}

void BlockBingo::turnRunnerAfterThrow(int afterdir)
{
    //afterdir 0:右上, 1:右下 2:左下 3:左上
    char buf[256];
    DIR current = mRunner->getDir();


    int dir4to8[] = {0,2,4,6};
    int slantdirto8[] = {1,3,5,7};

    int to = slantdirto8[afterdir];
    
    int diff = to - dir4to8[current];
    if(diff>4) diff=diff-8; 

    if (diff==1 || diff==-1) // 45度旋回スロー
        mRunner->turnRunner(diff);
    else if(diff>2)  // 135度旋回スロー
        mRunner->turnRunner(1);
    else if(diff<-2)
        mRunner->turnRunner(-1);

    sprintf(buf,"dir %d:%d->%d",current,afterdir,mRunner->getDir());
    msg_f(buf,0);
}

int BlockBingo::getGoalCircleId()
{
    return goal_num;
}

int BlockBingo::toExit()
{

    Runner* dummy = mRunner->makeClone();
    BlockPlace *start = dummy->getStart();
    mArea->cleanBlocks();

    int st = start->getNodeid();
    mDistance->setStart(st);
    mDistance->calcDistance();
    mDistance->calcTurncostAll(dummy->getDir());

    int exit[2];
    int exit_idx=0;
    mArea->getExitList(exit);
    int cost = mDistance->getLenAndTurnCost(exit[0]);
    int cost2=1000;
    if(exit[1]!=-1)
        cost2 = mDistance->getLenAndTurnCost(exit[1]);
    if(cost>cost2) 
        exit_idx=1;

    

    delete dummy;

    return exit[exit_idx];
}

void BlockBingo::getRoute(int node, int route[])
{
    static int line = 0;
    BlockPlace *start = mRunner->getStart(); // 走行体スタート位置取得
    DIR start_dir = mRunner->getDir();
    char buf[256];

    get_restored_block =-1; // 退避ブロックの位置初期化
  //  sprintf(buf,"getRoute:%d",mRunner->getDir());
  //   msg_f(buf,11);

 /*   Block *tmp = mArea->getBlock(node); // ブロックを取り除く
    if(tmp!=nullptr) // ブロックがあった
        mRunner->carryBlock(tmp); // ブロックをつかむ
        */
        
    int st = start->getNodeid(); // IDに変換
    mRunner->setStart(mArea->getBlockPlace(node)); // ゴールの位置をスタート位置へ

   //  msg_f("route 1",11);

    mDistance->setStart(st); // 最短経路のスタート
    mDistance->calcDistance();
    mDistance->calcTurncostAll(mRunner->getDir());
    // msg_f("route 2",11);
    /*if(start->getKind()==BPKIND::BLOCK) {
        mDistance->calcTurncostAllPostProcess(start->getNodeid(),start_dir);
            sprintf(buf,"route:%d",start_dir);
            //msg_f(buf,12);
    }*/

    if (start->getNodeid()!=node) 
        mDistance->setBlockinGoalCost(node);
   // if(line==13)
   //     mDistance->debugPrint();
   //  msg_f("route 3",11);

    mDistance->getRoute(st,node,route);
   /* if(line==13) {
     sprintf(buf,"getR %d %d %d %d-%d-%d",st,node,mArea->getBlockPlace(node)->getNodeid(),route[0],route[1],route[2]);
     msg_f(buf,10);
    }*/

   //  msg_f("route 4",11);

    int  toblock_endnode = node;

    //ゴールがブロックサークルの場合の処置。拾った後に抜ける
    if(mArea->getBlockPlace(toblock_endnode)->getKind()==BPKIND::BLOCK) {
        //sprintf(buf,"get %d %d %d",st,node,mArea->getBlockPlace(toblock_endnode)->getNodeid());
        //msg_f(buf,11);

        Block *bk = mArea->getBlock(toblock_endnode); //ブロックを拾う

        DIR dir = mDistance->getGoalDirection(toblock_endnode);
        int newnode = toblock_endnode;
        if(dir==DIR::N) 
            newnode-= 7;
        if(dir==DIR::S) 
            newnode += 7;
        if(dir==DIR::E) 
            newnode += 1;
        if(dir==DIR::W) 
            newnode -= 1;
        node = newnode;

        if(bk!=nullptr) {
            sprintf(buf,"black %d %d,%d,%d",toblock_endnode,bk->getColor(),dir,newnode);
            msg_f(buf,6);
        }

        mRunner->setStart(mArea->getBlockPlace(newnode));
        mArea->setBlock(newnode,bk); // ブロック移動する

        // ブロックサークルが最後の場合は抜けた後の黒線を最後にする？
        int cnt=0;
        for (cnt=0;route[cnt]!=999;cnt++) {
        }
        route[cnt]=node;
        route[cnt+1]=999;
    } 
    // ゴールが黒線退避の場合の処置.ブロックを運搬中は置いた後は下がる。拾った後は進む。 
    else if( mArea->getBlockPlace(toblock_endnode)->getKind()==BPKIND::LINE ) {
        int cnt=0;
        for (cnt=0;route[cnt]!=999;cnt++) {
        }
        // ブロックサークルからはそのまま
        if (cnt>1 && mArea->getBlockPlace(route[cnt-2])->getKind()==BPKIND::BLOCK ) {
            sprintf(buf,"from bk %d %d,%d",route[cnt-2],st,toblock_endnode);
            msg_f(buf,7);
           
            return;
        }
        Block *bk = mRunner->releaseBlock();
        DIR dir = mDistance->getGoalDirection(toblock_endnode);

        sprintf(buf,"get %d %d,%d",st,dir,toblock_endnode);
        msg_f(buf,7);

        char buf[256];
        if(bk!=nullptr) {
           /* sprintf(buf,"line %d? dir %d",line,dir);
            if(line==13)
                msg_f(buf,12);*/


            mArea->setBlock(toblock_endnode,bk); // ブロックを置く
            int newnode = toblock_endnode;
            if(dir==DIR::N) 
                newnode+= 7;
            if(dir==DIR::S) 
                newnode -= 7;
            if(dir==DIR::E) 
                newnode -= 1;
            if(dir==DIR::W) 
                newnode += 1;
            node = newnode;
        } else {
            Block *bk = mArea->getBlock(toblock_endnode); //ブロックを拾う

            int newnode = toblock_endnode;
            if(dir==DIR::N) 
                newnode-= 7;
            if(dir==DIR::S) 
                newnode += 7;
            if(dir==DIR::E) 
                newnode += 1;
            if(dir==DIR::W) 
                newnode -= 1;
            node = newnode;
            mArea->setBlock(node,bk); // ブロック移動する
            get_restored_block = toblock_endnode; // 退避ブロックの位置を記憶


        }
        mRunner->setStart(mArea->getBlockPlace(node));

        route[cnt]=node;
        route[cnt+1]=999;
     
       
    }

    if(st!=toblock_endnode) {
        DIR dir = mDistance->getGoalDirection(toblock_endnode); 
        char buf[256];
        if(node==12) {
       // sprintf(buf,"%d %d %d",st,node,dir);
        //msg_f(buf,10);
        }
        mRunner->setDir(dir);
    }
    Block *bk = mRunner->releaseBlock(); // 最後は必ずリリース


    //sprintf(buf,"%d:%d:%d %d %d %d %d %d %d %d ",line,mRunner->getDir(),route[0],route[1],route[2],route[3],route[4],route[5],route[6],route[7] );
    sprintf(buf,"%d:%d %d %d %d %d %d %d %d %d ",line,route[0],route[1],route[2],route[3],route[4],route[5],route[6],route[7],route[8] );
    if(line<=100) {
        line=4;
        msg_f(buf,((line++)+0)%13);
    }

}


int BlockBingo::currentNode()
{
    return mRunner->getStart()->getNodeid();
}

bool BlockBingo::finishCheck()
{
    char buf[256];
    sprintf(buf,"fin:%d",mArea->remainBlock());
    //msg_f(buf,12);
   
    return finish || mArea->remainBlock()==0;
}

void BlockBingo::decodeBt()
{

    int code0 = mBt->getRed();
    mArea->modifyBlockColor(code0%100,COLOR::RED);
    mArea->modifyBlockColor(code0/100,COLOR::RED);

    int code1 = mBt->getGreen();
    mArea->modifyBlockColor(code1%100,COLOR::GREEN);
    mArea->modifyBlockColor(code1/100,COLOR::GREEN);

    int code2 = mBt->getBlue();
    mArea->modifyBlockColor(code2%100,COLOR::BLUE);
    mArea->modifyBlockColor(code2/100,COLOR::BLUE);

    int code3 = mBt->getYellow();
    mArea->modifyBlockColor(code3%100,COLOR::YELLOW);
    mArea->modifyBlockColor(code3/100,COLOR::YELLOW);

    int code4 = mBt->getBlack();
    mArea->modifyBlockColor(code4%100,COLOR::BLACK);
    mArea->modifyBlockColor(code4/100,COLOR::BLACK);

    int code5 = mBt->getBonus();
    mArea->setBonusNo(code5);

    mArea->assignBlockList();

    char buf[256];
    sprintf(buf,"R%d G%d B%d Y%d K%d b%d",code0,code1,code2,code3,code4,code5);
    msg_f(buf,12);

}

COLOR BlockBingo::getBlockColor(int node)
{
    COLOR col=COLOR::NONE;
    Block *bk;
    BlockPlace *bp = mArea->getBlockPlace(node);
    if(bp!=nullptr) bk = bp->getBlock();
    if(bk!=nullptr) col = bk->getColor();

    return col;

}

COLOR BlockBingo::guessColor(int nodeid,hsv_t hsv)
{
    double h_r=H_RED_B;
    double h_g=H_GREEN_B;
    double h_b=H_BLUE_B;
    double h_y=H_YELLOW_B;

    COLOR col = getBlockColor(nodeid);
    if(col!=COLOR::NONE)
        return col;

    int *assign = mArea->getAssingPointer();
    
    COLOR ret;
    double min_diff = 1000;

    ret = COLOR::BLACK;   

    if(assign[0]<=1 && min_diff>getHueDistance(h_r,(double)hsv.h) && hsv.s>S_RED_B && hsv.v>1.0) {
        min_diff=getHueDistance(h_r,hsv.h);
        if(min_diff<120)
            ret = COLOR::RED;
    }

    if(assign[1]<=1 && min_diff>getHueDistance(h_g,(double)hsv.h) && hsv.s>S_GREEN_B && hsv.v>1.0) {
        min_diff=getHueDistance(h_g,hsv.h);
        if(min_diff<120)
        ret = COLOR::GREEN;
    }
    if(assign[2]<=1 && min_diff>getHueDistance(h_b,(double)hsv.h) && hsv.s>S_BLUE_B && hsv.v>1.0) {
        min_diff=getHueDistance(h_b,hsv.h);
        if(min_diff<120)
            ret = COLOR::BLUE;
    }
    if(assign[3]<=1 && min_diff>getHueDistance(h_y,(double)hsv.h) && hsv.s>S_YELLOW_B && hsv.v>1.0) {
        min_diff=getHueDistance(h_y,hsv.h);
        if(min_diff<120)
            ret = COLOR::YELLOW;
    }

    if(assign[4]<=1 &&  (hsv.s<0.1 && hsv.v<0.1 ||  hsv.s==1.0 && hsv.v==1.0 )) {
         ret = COLOR::BLACK;   
    }

    mArea->modifyBlockColorByNodeId(nodeid,ret);
    mArea->assignBlockList();

    char buf[256];
    char msg[] = {'R','G','B','Y','K'};
    sprintf(buf,"COLOR:%c %3.0f,%2.1f,%2.1f",msg[ret],hsv.h,hsv.s,hsv.v);
    msg_f(buf,0);

    return ret;
}

double BlockBingo::getHueDistance(double ang1,double ang2)
{
    double diff = fabs(ang1-ang2);
    diff = diff>180?360-diff:diff;

    return diff;
}

int BlockBingo::getRestoredBlockMode()
{
    return get_restored_block;
}