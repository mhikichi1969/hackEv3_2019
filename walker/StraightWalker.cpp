#include "StraightWalker.h"

StraightWalker::StraightWalker(ev3api::Motor& leftWheel,
           ev3api::Motor& rightWheel,    
           Odometry* odo,
            SpeedControl *scon) 
:SimpleWalker(leftWheel,rightWheel,odo,scon),
mState(UNDEFINED)
{
    //msg_f("StraightWalker create",1);
    mPID = new HPID();
    setPID();

    mDiffRecord=0.0f;
    mTargetLength=0.0f;
    mPWM = 0;

}
StraightWalker::~StraightWalker()
{
    delete mPID;
}

/* 未定義状態
 * 初期処理に移行
*/
void StraightWalker::execUndefined() 
{
    mState = INIT;
}

/* 初期処理
 * 走行に必要な値を指定後に走行処理に移行
*/
void StraightWalker::initial()
{

    mState = WALKING;
}

/* 走行処理
 * 指定した距離を走るか指定した一定時間の経過で終了
*/
void StraightWalker::execWalking()
{
    mDiffRecord = mOdo->getWheelCountDiffFromRecord(1.0);     //左モータの回転数から右モータの回転数を引いた角度（°）
    float turn = mPID->getOperation(mDiffRecord);
    if(mBackFlag){
        SimpleWalker::setCommandV(mPWM,turn);           //後退用
        //SimpleWalker::setCommand(mPWM,turn);           //
    }else{
        SimpleWalker::setCommandV(mPWM,turn);           //前進用
        //SimpleWalker::setCommand(mPWM,turn);           //
    }
    SimpleWalker::run();
    
    
   /* char buf[256];
    sprintf(buf,"Straight:diff:%f",mDiffRecord);
    msg_f(buf,1);
    sprintf(buf,"Straight:turn:%f",turn);
    msg_f(buf,2);*/

    //sprintf(buf,"Straight:cnt:%f",cnt);
    //msg_f(buf,5);
    

}

/* 終了処理
 * モーターを停止
*/
void StraightWalker::end()
{
    SimpleWalker::setCommand(0,0);
    SimpleWalker::run();
}

/* 直進処理を管理
*/
void StraightWalker::run()
{
    //msg_f("StraightWalker run",2);
    
    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case INIT:
            initial();
            break;
        case WALKING:
            execWalking();
            break;
        case END:
            end();
            break;

    }
}

/* 直進する距離とPIDなどの値を指定
 * 直進前に指定する必要がある
 * 
 * length           :float      :距離（cm）
*/
void StraightWalker::setParam(float length)
{
    setTargetLength(length);
    setPID();
}

/* PID計算結果の上限値を指定
 * PIDを計算した結果が上限値を超えた場合には上限値が適用される
 * 
 * limit            :double     :上限値
*/
void StraightWalker::setLimit(double limit) 
{
    mPID->setLimit(limit);
}

/* 直進に使用するPID値などを指定
 * 
*/
void StraightWalker::setPID()
{
    mPID->setTarget(0);
    mPID->resetParam();

    //
    mPID->setKp(0.3f); //0.45f 0.25f
    mPID->setKi(0.0f);
    mPID->setKd(0.01f);

    //mPID->setKi(20.0f);
    //mPID->setKi(60.0f);
    //mPID->setKd(2.5f);
    //mPID->setLimit(15);
}

/* 直進時PWMを指定
 * 
 * pwm              :int        :直進時のPWM値
*/
void StraightWalker::setPWM(int pwm) 
{
    mPWM = pwm;
}

/* スタートから現在地までの距離を保存
 * 
 * length           :float      :距離（cm）
*/
void StraightWalker::setCurrentLength(float length)
{
    mCurrentLength = length;
}

/* 直進する距離を指定
 * 
 * length           :float      :距離（cm）
*/
void StraightWalker::setTargetLength(float length)
{
    mTargetLength = length;

}

/* 状態を未定義に変更
*/
void StraightWalker::setStateUndefined()
{
    mState = UNDEFINED;
}

/* 直進が終了したかどうかを返す
 * 
 * return           :1          :完了
 *                  :0          :未完了
*/
int StraightWalker::isEnd() 
{
    return mState==END;
}

/* 前進か後退化を判別するフラグを設定する
 * 
 * f                :true       :後退
 *                  :false      :前進
*/
void StraightWalker::setFBFlag(bool f)
{
    mBackFlag = f;

}






