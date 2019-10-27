#include "CompositeSection.h"
#include "Const.h"
#include "util.h"

CompositeSection::CompositeSection(Judge *judge,
                                   LineTracer *tracer,
                                   StraightWalker *straight,
                                   Turn *turn,
                                VirtualTracer *vt,
                                   ArmControl *arm,
                                   SDFile *sdfile):
                  Section(judge, tracer,straight,turn,vt),
                  //mState(EXEC_UNDEF),
                  mArm(arm),
                  mSd(sdfile)
{
    mAction = Flag::RUN_UNDEF;
    mTmpS = {(double)Flag::RUN_UNDEF,0,0,0,0,0,0,0};
    mTmpP = {0,0,0,0,Flag::RUN_UNDEF,Flag::END_UDF};
    mState = EXEC_UNDEF;
    //mSC = new SectionCreate();

}

CompositeSection::~CompositeSection()
{
    //delete mSC;
}


bool CompositeSection::run()
{
    //msg_f("CompositeSection::run",1);
    switch(mState)
    {
        case EXEC_UNDEF:
            execUndefined();
            break;
        case EXEC_INIT:
            execInit();
            break;
        case EXEC_RUN:
            execRun();
            break;
        case EXEC_END:
            //msg_f("CS:END",6);
            mState = EXEC_UNDEF;  //非デバッグ用
            return true;
    }

    return false;
}

/* 未定義状態
 * パラメータのインデックスを初期化後に初期処理に移行
*/
void CompositeSection::execUndefined()
{
    //msg_f("CompositeSection",2);
    param_idx=0;
    save_idx=0;
    hsv_idx=0;
    hsvArray[hsv_idx].h = 0;
    hsvArray[hsv_idx].s = 0;
    hsvArray[hsv_idx].v = 0;
    mAction = Flag::RUN_LINE;

    //mJudge->recordCount();        //非デバッグ用
    mState = EXEC_INIT;

}

/* 初期処理
 * アクションごとにパラメーターの読み込みを行う
 * 最後ではない場合には走行に必要な初期処理の後に各走行処理に移行
*/
void CompositeSection::execInit()
{
    char buf[256];
    mJudge->resetLength();  //進行距離リセット
    
    switch (mAction){
        case Flag::RUN_SPEED:     //タイムアタックを行う場合
            mTmpS = mParamS[param_idx]; 
            mAction = (Flag::Method)mTmpS.fwd;

            //char buf[256];
            //sprintf(buf,"%2.0f,%1.1f,%3.1f,%3.1f,%3.1f %2.1f,%2.1f",mTmpS.fwd,mTmpS.target,mTmpS.kp,mTmpS.ki,mTmpS.kd,mTmpS.curve,mTmpS.ckp);
            //msg_f(buf,10);

            break;
    
        case Flag::RUN_ONLINE:       //黒線寄りにライントレースを行う場合
        case Flag::RUN_LINE:         //エッジ上でライントレースを行う場合
        case Flag::RUN_STRAIGHT:     //直進を行う場合
        case Flag::RUN_TURN:         //旋回を行う場合
        case Flag::RUN_VIRTUAL:
        case Flag::RUN_THROW:        //ブロックをアームで投げる場合
        case Flag::RUN_ARM:          //アームをPID制御で操作する場合
        case Flag::RUN_COLOR:        //ブロック色を確認する場合
        case Flag::RUN_SAVE:         //ブロック色を保存する場合
        case Flag::RUN_RECORD:       //レコードカウントを行う場合
        case Flag::RUN_EDGE:         //エッジ制御を行う場合
            //msg_f("CompositeSection:not speed",2);
            //char buf[256];
            //sprintf(buf,"CS:idx %d",param_idx);
            //msg_f(buf,5);

            mTmpP = mParamP[param_idx]; 
            mAction = mTmpP.runFlag;
            mJudge->setParam(mTmpP.fwd, mTmpP.target, mTmpP.len, 
                             mTmpP.turn, mTmpP.runFlag, mTmpP.endFlag);
            //mAction = mTmpP->runFlag;
            //mJudge->setParam(mTmpP->fwd, mTmpP->target, mTmpP->len, 
            //                 mTmpP->turn, mTmpP->runFlag, mTmpP->endFlag);
            //sprintf(buf,"CS:TMP:%d : runFlag:%d",(int)mTmpP.runFlag);
            //msg_f(buf,5);


            break;
            
        case Flag::RUN_UNDEF: //未定義状態
        default:
            break;
    }


    //全てを読み終わった場合には終了
    if(mAction==Flag::RUN_END) {
        //smsg_f("CS:ACTION:END",12);
        mState = EXEC_END;
        mAction = Flag::RUN_UNDEF;
        return;
    }

    switch (mAction){
        case Flag::RUN_SPEED:     //タイムアタックを行う場合
            //((LineTracer*)mSimpleWalker)->resetParam();
            //float speed,float target,float kp, float ki, float kd,float angleTarget,float angleKp
            //float speed,float target,float kp, float ki, float kd,float Curve(1.0),float bias(1.0)
            ((LineTracer*)mSimpleWalker)->setParam((float)mTmpS.fwd, (float)mTmpS.target, (float)mTmpS.kp, (float)mTmpS.ki, (float)mTmpS.kd,  (float)mTmpS.curve, (float)mTmpS.ckp);
            // ((LineTracer*)mRunStyle)->resetParam();
            ((LineTracer*)mSimpleWalker)->setBias(0);

            //recordCount();

            //終了条件、フラグ管理を追加し、距離、色、角度、などで判定予定
            //mJudge->setValue(mTmpS.len);    //終了条件、距離

            // mSd->open("/ev3rt/apps/Lbase.csv");
            //mState = EXEC_TRACE;
            mState = EXEC_RUN;
            
            break;
    
        case Flag::RUN_ONLINE:     //黒線寄りにライントレースを行う場合
            //msg_f("CS:ACTION:ONLINE",3);
            setParamL((float)mTmpP.fwd, -0.5f);

            mState = EXEC_RUN;

            break;

        case Flag::RUN_LINE:     //エッジ上でライントレースを行う場合
            //msg_f("CS:ACTION:LINE",3);
            setParamL((float)mTmpP.fwd, 0.0f);

            //char buf[256];
            //sprintf(buf,"CS:LINE %2.0f",mTmpP.fwd);
            //msg_f(buf,5);

            mState = EXEC_RUN;

            break;
    
        case Flag::RUN_STRAIGHT:  //直進を行う場合
            //msg_f("CS:ACTION:STRAIGHT",3);
            mStraightWalker->setPID();
            //mStraightWalker->setLimit(mTmpP.fwd);       //PIDのpwm上限値、FWDと同じにしなければ回転し続ける可能性あり
            mStraightWalker->setLimit(mTmpP.fwd>0?mTmpP.fwd*0.9:-mTmpP.fwd*0.9);    //PIDのpwm上限値
            mStraightWalker->setPWM(mTmpP.fwd);         //直進時のPWM値
            mStraightWalker->setFBFlag(mTmpP.len < 0);         //距離の正負から前進か後退か判断

            //sprintf(buf,"CS:S:target%f",mTmpP.target);
            //msg_f(buf,6);
            mState = EXEC_RUN;

            break;

        case Flag::RUN_TURN:      //旋回を行う場合
            //msg_f("CS:ACTION:TURN",3);
            mTurn->setFwd(mTmpP.fwd);
            mTurn->setParam(mTmpP.target);

            /* //PID制御していた時のコード
            if(mTmpP.fwd>0){
                //ブロックを持つ場合、旋回値は固定
                if(mTmpP.target > 0){
                    mTurn->setTurn(mTmpP.turn);
                }else{
                    mTurn->setTurn(-mTmpP.turn);
                }
            }else{
                //ブロックを持たない場合、旋回値はPID制御
                mTurn->setLowPWM(2);
                //mTurn->setLimit(30);
            }*/

            //旋回値を低くしすぎるとジャイロが反応しなくなるため、不具合発生の可能性あり
            //turn:5までは正常動作確認済み
           // if(mTmpP.target > 0){
            if(mJudge->getTurnDirection(mTmpP.target)==1) {
                //右旋回の場合
                mTurn->setTurn(mTmpP.turn);
            }else{
                //左旋回の場合
                mTurn->setTurn(-mTmpP.turn);
            }

            mJudge->setPermitAngle(1);      //誤差許容角度

            mState = EXEC_RUN;

            break;
        case Flag::RUN_VIRTUAL:
            setParamVirtual(mTmpP.fwd,mTmpP.turn);
            mState = EXEC_RUN;
            break;
        case Flag::RUN_THROW:     //ブロックを投げる場合
            //msg_f("CompositeSection:THROW",2);
            mArm->setPwm(mTmpP.fwd);
            //mArm->setPwm(mTmpP->fwd);
            mArm->changeStateRun();

            mState = EXEC_RUN;

            break;

        case Flag::RUN_ARM:         //アームをPID制御で操作する場合
            //msg_f("CompositeSection:ARM",2);
            mArm->setAngle(mTmpP.target);

            mState = EXEC_RUN;

            break;

        case Flag::RUN_COLOR:       //ブロック色を確認する場合
            //msg_f("CompositeSection:COLOR",2);
            hsv_idx=0;
            hsvArray[hsv_idx].h = 0;
            hsvArray[hsv_idx].s = 0;
            hsvArray[hsv_idx].v = 0;

            mState = EXEC_RUN;

            break;

        case Flag::RUN_SAVE :   //ブロック色を保存する場合
        case Flag::RUN_RECORD:  //レコードカウントを行う場合
        case Flag::RUN_EDGE:    //エッジ制御を行う場合
            mState = EXEC_RUN;
            break;

        case Flag::RUN_UNDEF: //未定義状態
        default:
            //msg_f("CS:ACTION:Nothing",3);
            //char buf[256];
            //sprintf(buf,"CS:idx%d, RUN%d",param_idx,(int)mAction);
            //msg_f(buf,6);

            //sprintf(buf,"CS:E:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",(int)mParamP[10].runFlag,(int)mParamP[11].runFlag,(int)mParamP[12].runFlag,(int)mParamP[13].runFlag,(int)mParamP[14].runFlag,(int)mParamP[15].runFlag,(int)mParamP[16].runFlag,(int)mParamP[17].runFlag,(int)mParamP[18].runFlag,(int)mParamP[19].runFlag);
            //msg_f(buf,12);
            break;
    }
}

/* 
*/
void CompositeSection::execRun()
{
    //msg_f("Composite:execRun",1);

    switch (mAction){
        case Flag::RUN_SPEED:       //タイムアタックを行う場合
            //msg_f("CS:EXEC:RUN_LINE",9);
            ((LineTracer*)mSimpleWalker)->run();
            break;
    
        case Flag::RUN_ONLINE:      //黒線寄りにライントレースを行う場合
        case Flag::RUN_LINE:        //エッジ上でライントレースを行う場合
            //msg_f("CS:EXEC:RUN_LINE",3);
            //ev3_speaker_play_tone(NOTE_F4,10);
            ((LineTracer*)mSimpleWalker)->run();
            break;

        case Flag::RUN_STRAIGHT:    //直進を行う場合
            //msg_f("CS:EXEC:RUN_STRAIGHT",3);
            mStraightWalker->run();
            break;

        case Flag::RUN_TURN:        //旋回を行う場合
            //msg_f("CS:EXEC:TURN",3);
            mTurn->run();
            break;
        case Flag::RUN_VIRTUAL:
            mVirtualTracer->run();
            break;

        case Flag::RUN_RECORD:      //レコードカウントを行う場合
            //msg_f("CS:EXEC:RECORD",3);
            mJudge->recordCount();
            execNextParam();
            return;
            //break;

        case Flag::RUN_EDGE:        //エッジ制御を行う場合
            //msg_f("Composite:EdgeChange",3);

            bool edge;
            if(mTmpP.target == Turn::LEFT){
                //msg_f("Composite:EdgeChange:LEFT",3);
                edge = LineTracer::LEFTEDGE;        
            }else{
                //msg_f("Composite:EdgeChange:RIGHT",3);
                edge = LineTracer::RIGHTEDGE;        
            }
            ((LineTracer*)mSimpleWalker)->setEdgeMode(edge);

            execNextParam();
            return;
            //break;

        case Flag::RUN_THROW:       //ブロックを投げる場合
            //msg_f("Composite:THROWBLOCK",3);
            //mJudge->addTargetParam(-1.0d); //目標角度に到達できなかった場合は徐々に目標角度を下げていく
            mJudge->addTargetParam(-0.5d); //目標角度に到達できなかった場合は徐々に目標角度を下げていく

            break;

        case Flag::RUN_ARM:         //アームをPID制御で操作する場合
            //msg_f("Composite:ARM",3);
            mJudge->addTargetParam(-0.5d); //目標角度に到達できなかった場合は徐々に目標角度を下げていく
            break;

        case Flag::RUN_COLOR:       //ブロック色を確認する場合
            //msg_f("Composite:COLOR",2);
            mJudge->setHSV();

            hsvArray[hsv_idx] = mJudge->getHSV();
            hsv_idx++;

            break;

        case Flag::RUN_SAVE:        //ブロック色を保存する場合
            saveHSV();
            execNextParam();
            return;

        case Flag::RUN_UNDEF:       //未定義状態
        default:
            break;
    }

    if(mJudge->isOK()) {
        execNextParam();
    }
}

/* 
*/
void CompositeSection::execNextParam()
{
    param_idx++;
    mState = EXEC_INIT;
    mJudge->resetParam();
}

/* ライントレース用のパラメータを設定
 * 
*/
void CompositeSection::setParamL(float fwd, float border)
{
    //ev3_speaker_play_tone(NOTE_F4,100);
    double kp = CARRY_KP;
    double ki = CARRY_KI;
    double kd = CARRY_KD;

    ((LineTracer*)mSimpleWalker)->resetLinePid();
    ((LineTracer*)mSimpleWalker)->setBias(0);
    ((LineTracer*)mSimpleWalker)->setParam(fwd, border, kp, ki, kd, 1.0f, 1.0f);

    //char buf[256];
    //sprintf(buf,"CS:SET_PARAM_L %2.0f",fwd);
    //msg_f(buf,5);
}

void CompositeSection::setParamVirtual(double fwd, double center)
{
    double kp = CARRY_KP;
    double ki = CARRY_KI;
    double kd = CARRY_KD;

    double vkp=7.5; //2.5
    double vki=2.0; //2.5
    double vkd=5.0; //2.6

    mVirtualTracer->setDirectPwmMode(true);
    mVirtualTracer->resetPid();
    mVirtualTracer->setParam(fwd, 0, center, vkp,vki,vkd );
}

/*
 * 
*/
void CompositeSection::setSection(PParam p, int i)
{
        //char buf[256];
        //sprintf(buf,"CS:sec:i: %d",i);
        //msg_f(buf,8);

        mParamP[i] = p;
}

/*
 * 
*/
void CompositeSection::setActionUndefined()
{
        mState = EXEC_UNDEF;
}

/*
 * 
*/
void CompositeSection::saveHSV()
{
    int a[hsv_idx]; //仮保存配列
    int i=0;        //ループ用
    int j=0;        //仮保存配列用
    double n=0;     //計算用

    hsv_t hsv = hsvArray[i];
    i++;

    while(i<hsv_idx){   //sが一番高い値のみを仮保存
        if(hsv.s<hsvArray[i].s){
            hsv = hsvArray[i];
            j = 0;
            a[j] = i;
            j++;
        }else if(hsv.s==hsvArray[i].s){ //sが同率の値を保存
            a[j] = i;
            j++;
        }
        i++;
    }

    if(hsv_idx>0 && hsv.s>0.3 && hsv.v>0.3){  //hsvの値を決定
        i=0;
        while(i<j){ 
            n += hsvArray[a[i]].h;
            i++;
        }
        mHsv[save_idx].h = n/j;  //sが最高の値から算出したhの平均値
        mHsv[save_idx].s = hsvArray[0].s;
        mHsv[save_idx].v = hsv.v;
    }else{  //初期値または黒（？）
        mHsv[save_idx].h = 0;
        mHsv[save_idx].s = 0;
        mHsv[save_idx].v = 0;
    }
    
    save_idx++;

    /*char buf[256];
    sprintf(buf,"CS:SAVE:idx %d",save_idx);
    msg_f(buf,6);
    sprintf(buf,"CS:SAVE:hsv %3.1f %3.1f %3.1f",mHsv[0].h,mHsv[0].s,mHsv[0].v);
    msg_f(buf,7);
    */
}

/*
 * 
*/
hsv_t CompositeSection::getHSV()
{
    return mHsv[0];
}

/*
 * 
*/
hsv_t CompositeSection::getHSV(int i)
{
    return mHsv[i];
}