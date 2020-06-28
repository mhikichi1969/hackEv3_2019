#include "Runner.h"
#include "math.h"
#include "Area.h" //相互参照
#include "util.h"

Runner::Runner():
    dir(DIR::W),
    start(nullptr),
    end(nullptr),
    block(nullptr)
{
	setCircleBefore(true);
	mOnLine = false;
}

void Runner::setStart(BlockPlace *bp)
{
    start = bp;
	mNext = bp;
}
void Runner::setEnd(BlockPlace *bp)
{
    end = bp;
}
BlockPlace* Runner::getStart()
{
    return start; 
}
BlockPlace* Runner::getEnd()
{
    return end;
}

void Runner::setDir(DIR d)
{
    dir = d;
}

void Runner::carryBlock(Block *bk)
{
    block = bk;
}

Block* Runner::checkBlock()
{
    return block;
}

Block*  Runner::releaseBlock()
{
	Block *tmp = block;
    block = nullptr;
	return tmp;
}

// turn : 右方向+  左方向-
int Runner::turnRunner(int turn) {
    int dir_num = (int)dir;
    dir_num = dir_num+turn;
    if (dir_num>3) dir_num=dir_num-4;
    if (dir_num<0) dir_num=dir_num+4;
    dir = (DIR)dir_num;
    //return (int)fabs(turn);
    return (int)turn;
}

/* 座標から回転方向を算出　*/		//移動コスト計算用
int Runner::checkDirection(int from_[],int to[]) {
        int ret=0;
		if (dir==DIR::N) {
			if (from_[1]>to[1])
				ret = turnRunner(-1);
			else if(from_[1]<to[1])
				ret = turnRunner(1);
			else if (from_[0]<to[0])
				ret = turnRunner(2);
        }
		if (dir==DIR::E) {
			if (from_[0]>to[0])
				ret = turnRunner(-1);
			else if (from_[0]<to[0])
				ret = turnRunner(1);
			else if (from_[1]>to[1])
				ret = turnRunner(2);
        }
		if (dir==DIR::S) {
			if (from_[1]>to[1])
				ret = turnRunner(1);
			else if(from_[1]<to[1])
				ret = turnRunner(-1);
			else if (from_[0]>to[0])
				ret = turnRunner(2);
        }
		if (dir==DIR::W) {
			if (from_[0]>to[0])
				ret = turnRunner(1);
			else if (from_[0]<to[0])
				ret = turnRunner(-1);
			else if (from_[1]<to[1])
				ret = turnRunner(2);
        }
		return (int)fabs(ret);
}

DIR Runner::getDir()
{
    return dir;
}

Runner *Runner::makeClone()
{
    Runner *clone= new Runner();
    clone->dir = dir;

    clone->start=start;
    clone->end=end;

    clone->block = block;

    return clone;
}



//intから交点サークル保存、主にSectionCreateから使用
void Runner::setNext(int next)
{
    start = mNext;
    mNext = mArea->getBlockPlace(next);
}

//BrockPlaceから交点サークル保存、主にAreaから使用
void Runner::setNext(BlockPlace *bp)
{
    mNext = bp;
}

void Runner::setArea(Area *a)
{
    mArea = a;
}

int Runner::calcRoute()
{
    //msg_f("RUN:calcroute",7);
    start->getNodeXY(sbp);
    mNext->getNodeXY(nbp);

   /* 
    static char buf[256];
    sprintf(buf,"RUN:CAL:sbp : %d,%d",sbp[0], sbp[1]);
    msg_f(buf,0);
    sprintf(buf,"RUN:CAL:nbp : %d,%d",nbp[0], nbp[1]);
    msg_f(buf,1);*/
    

    //msg_f("RUN:calcroute:END",8);
    return (calcDirection(sbp,nbp));

}

/* 座標から回転方向を算出　*/		//走行体向きの把握用、右：１、後：２、左：－１
int Runner::calcDirection(int from_[],int to[]) {
        int ret=0;
		if (dir==DIR::N) {
			if (from_[1]>to[1])
				ret = turnRunner(-1);
			else if(from_[1]<to[1])
				ret = turnRunner(1);
			else if (from_[0]<to[0])
				ret = turnRunner(2);
        }
		if (dir==DIR::E) {
			if (from_[0]>to[0])
				ret = turnRunner(-1);
			else if (from_[0]<to[0])
				ret = turnRunner(1);
			else if (from_[1]>to[1])
				ret = turnRunner(2);
        }
		if (dir==DIR::S) {
			if (from_[1]>to[1])
				ret = turnRunner(1);
			else if(from_[1]<to[1])
				ret = turnRunner(-1);
			else if (from_[0]>to[0])
				ret = turnRunner(2);
        }
		if (dir==DIR::W) {
			if (from_[0]>to[0])
				ret = turnRunner(1);
			else if (from_[0]<to[0])
				ret = turnRunner(-1);
			else if (from_[1]<to[1])
				ret = turnRunner(2);
        }
		return ret;
}

COLOR Runner::getNextColor() {
    return ((CrossCircle*)mNext)->getColor();
}

int Runner::getNextNodeId(){
	return mNext->getNodeid();
}

BlockPlace* Runner::getNextBlockPlace(){
	return mNext;
}

bool Runner::isCircleBefore(){
	return mCircleBefore;
}

void Runner::setCircleBefore(bool b){
	mCircleBefore = b;
	mOnLine = false;
}
// 必ず　setCircleBefore　の後に実行すること
void Runner::setOnLine(bool b){
	mOnLine = b;
}

bool Runner::isOnLine()
{
	return mOnLine;
}

