#include "SceneWalker.h"

#include "HPolling.h"
#include "Odometry.h"
#include "LineTracer.h"
#include "SDFile.h"
#include "SectionJudge.h"
#include "Const.h"

#include "util.h"

extern HPolling *gHPolling;
extern Odometry *gOdo;
extern LineTracer *gLineTracer;
extern VirtualTracer *gVirtualTracer;

extern SDFile * gSDFile;
extern Judge *gJudge;
extern SectionJudge *gSectionJudge;
extern Turn *gTurn;
extern StraightWalker *gStraightWalker;
extern ArmControl *gArmControl;
extern HBTtask *gBTcomm;


SceneWalker::SceneWalker(HCalibrator *cal,
                    Starter *start):
                    mCal(cal),
                    mStarter(start),
                    mState(UNDEFINED)
{


}                  

void SceneWalker::run(){

    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case INIT:
            execInit();
            break;
        case CALIBRATION:
            execCalibration();
            break;
        case START:
            execStarter();
            break;
        case TRACE:
            execTracer();
            break;
        case INITTOBINGO:
            initToBingo();
            break;
        case BINGO:
            execToBingo();
            break;

        case WAIT:
            execWait();
            break;
        case TOBLOCK:
            execToBlock();
            break;
        case SEARCHCOL:
            execSearchColor();
            break;
        case CARRYBLOCK:
            execCarryBlock();
            break;
        case TOPARKING:
            execExit();
            break;
        case COMPOSITE:
            execComposite();
            break;
        case END:
            execEnd();
            break;
        case DEBUG:
            execDebug();
            break;
    }

    
}

void SceneWalker::execUndefined()
{
#if RUNNER_NO==0
    msg_f("build for MS-08",0);
#elif RUNNER_NO==1
    msg_f("build for MS-18",0);
#endif
    mState = INIT;
}

void SceneWalker::execInit()
{

    
    mSSection = new SpeedSection(
                        gSectionJudge,
                        gLineTracer,
                        gStraightWalker,
                        gTurn,
                        gVirtualTracer,
                        gArmControl,
                        gSDFile);

    
    mCSection = new CompositeSection(
                        gJudge,
                        gLineTracer,
                        gStraightWalker,
                        gTurn,
                        gVirtualTracer,
                        gArmControl,
                        gSDFile);

    

    mState = CALIBRATION;
    //mState = TOBLOCK;
}
void SceneWalker::execCalibration()
{
    mCal->run();
    if(mCal->isEnd()) {
        mState = START;    
        //mState = END;    //デバッグ用
        int course = mCal->getCodeNum(0)%2; // 偶数ならLコース、奇数ならRコース
        int sel = mCal->getCodeNum(1);
        ((SpeedSection*)mSSection)->setCourse(course);
        ((SpeedSection*)mSSection)->selectParamNo(sel);
        mTimer = new Timer(course);
        mBlockBingo = new BlockBingo(gBTcomm, mTimer,course); //デバッグ用
        mSC = new SectionCreate();
        mSC->setCourse(course);

    }
}
void SceneWalker::execStarter()
{
    mStarter->run();
    if(mStarter->isTouched()) {
        mTimer->setStart(); // タイムアウトタイマーセット
        mSC->setCompositeSection(mCSection);
       // mState = TOBLOCK;    //デバッグ用
        //mState = COMPOSITE;   //デバッグ用
       // mState = TRACE;   //非デバッグ用
        int start_select = mCal->getCodeNum(2)%3; // 0ならスピードコース、1ならビンゴ,2はデバッグ
        switch(start_select) {
            case 0:
                mState = TRACE;
            break;
            case 1:
                mState = INITTOBINGO;
            break;
            case 2:
                mState = DEBUG;
            break;
        }
        ev3_stp_cyc(EV3_CYC_DEVICE_ERROR);
   }
}

void SceneWalker::execTracer()
{
    //msg_f("execTracer",1);
  /* int course = mCal->getCodeNum(0)%2; // 偶数ならLコース、奇数ならRコース
    if (course==0) 
        mState = SEARCHCOL; 
    else
        mState = TOBLOCK;
    return; //debug
    */

    if(mSSection->run()) {
        //mState = COMPOSITE;       
       // mBlockBingo->decodeBt(); // 本番はここで色をセット
        //mState = TOBLOCK;
        mState = INITTOBINGO;
    }
}




void SceneWalker::initToBingo() 
{
    PParam p;
    int i=0;
    do {
        p = mParamSS[i];
        ((CompositeSection*)mCSection)->setSection(p,i);
        i++;
    } while(p.endFlag!=Flag::END_UDF);

    mBlockBingo->decodeBt(); 

    mState = BINGO;
}

void SceneWalker::execToBingo()
{
    if(mCSection->run()) {
        mState = WAIT;
    }
}

void SceneWalker::execWait()
{
    static int cnt=0;
    while(cnt++<100) {
        return;
    }

    gHPolling->newGyro();
    gVirtualTracer->setGyroMode(true);

    ((CompositeSection*)mCSection)->setActionUndefined();
    int course = mCal->getCodeNum(0)%2; // 偶数ならLコース、奇数ならRコース
/*
    if (course==0) 
        mState = SEARCHCOL; 
    else
        mState = TOBLOCK;    */
    mState = TOBLOCK;

}


void SceneWalker::execToBlock()
{
    static int cnt=0;

    //ブロック無しをチェック
    if(mBlockBingo->finishCheck()){
      //  msg_f("no block",0);
        mState=TOPARKING;
        return;
    }

    int node = mBlockBingo->selectCarry();
    //タイムアウトをチェック
    if(mBlockBingo->finishCheck()){
     //   msg_f("time out",0);

        mState=TOPARKING;
        return;
    }


     char buf[256];
    sprintf(buf,"t:%d(%d)->",node,mBlockBingo->getBlockColor(node));
    msg_f(buf,11);
   
    int col = mBlockBingo->getBlockColor(node);

    int route[20];
    mBlockBingo->getRoute(node,route);
    mSC->setGetResotoredBlockMode(mBlockBingo->getRestoredBlockMode());
    //mSC->setRoutes(route,false); //旧：ToBlockから複合区間に経路設定

            //mSC->setRoutes(route,-5); //ToBlockから複合区間に経路設定
    
    if(col!=COLOR::NONE){
        //msg_f("COLOR:OK",1);
        mSC->setRoutes(route,-5); //ToBlockから複合区間に経路設定
    }else{
        //msg_f("COLOR:NONE",1);
        mSC->setRoutes(route,-3); //ブロックまで移動後に色取得
    }

    mBState=mState;
    mState = COMPOSITE;

   // mState = SEARCHCOL; //debug

    /*
    if(cnt++==1) 
        mState=END;
    */

}

void SceneWalker::execSearchColor()
{

    mState = CARRYBLOCK;    //デバッグ用
    //mBState=mState;
    //mState=COMPOSITE;
}

void SceneWalker::execCarryBlock()
{
    char buf[256];

    static int cnt=0;

    //ブロック色が不明な場合は取得したブロック色をエリアにセットする
    hsv_t hsv = ((CompositeSection*)mCSection)->getHSV();

    int st = mBlockBingo->currentNode();

    mBlockBingo->guessColor(st,hsv);

    int node = mBlockBingo->carryBlock(st);
   int goal = mBlockBingo->getGoalCircleId();

    sprintf(buf,"c:%d->%d(%d)",st,node,goal);
   /* if(st==4)
        msg_f(buf,11);*/

  int route[20];
    mBlockBingo->getRoute(node,route);

 /*   sprintf(buf,"%d-%d-%d",route[0],route[1],route[2]);
    if(st==4)
        msg_f(buf,12);*/

    //mSC->setRoutes(route,true); //旧：CarryBlockから複合区間に経路設定
    mSC->setRoutes(route,goal); //CarryBlockから複合区間に経路設定
    //mState=COMPOSITE;

    
    /*if(cnt++== 8) 
        mState=END;*/
    
    /*if(mBlockBingo->finishCheck()){
        mState=TOPARKING;
    }else{*/
        mBState=mState;
        mState=COMPOSITE;
    //}

   // mState=TOBLOCK; //debug


}

void SceneWalker::execExit()
{
    msg_f("exit ",12);
    int node = mBlockBingo->toExit();
    int route[20];
    mBlockBingo->getRoute(node,route);
    msg_f("exit 2",12);
  
    mSC->setRoutes(route,-2); //CarryBlockから複合区間に経路設定

    mBState=mState;
    mState=COMPOSITE;
    //mState=END;
 
}

void SceneWalker::execComposite()
{
    //msg_f("execComposite",1);    
    if(mSC->run()) {
        switch (mBState){
            case TOBLOCK:
                mState = SEARCHCOL;
                break;
            case SEARCHCOL:
                mState = CARRYBLOCK;
                break;
            case CARRYBLOCK:
                mState = TOBLOCK;
                //mState = END; //デバッグ用
                break;
            case TOPARKING:
            case DEBUG:
                mState = END;
                break;
                
            default:
                msg_f("SW:execComposite_ERR",1);
                break;
        }
    }
}


void SceneWalker::execDebug()
{
    //msg_f("execDebug",1);    
    //int i=0;

    if(mSC->run()) {
        if(mDebA[deb_idx][0] == 999){
            mState=END;
            return;
        }
        mSC->setRoutes(mDebA[deb_idx],mDebN[deb_idx]);
        deb_idx++;
    }else if(deb_idx==0){   //単独デバッグ
        mSC->setRoutes(mDebA[0],mDebN[0]);
        deb_idx++;
        //deb_idx++;
    }else if(deb_idx==4){   //連続デバッグ
        mSC->setRoutes(mDebA[deb_idx],mDebN[deb_idx]);
        deb_idx++;
    } 

}

void SceneWalker::execEnd()
{
    gArmControl->setPwm(0);
}