#include "Turn.h"

Turn::Turn(ev3api::Motor& leftWheel,
           ev3api::Motor& rightWheel,    
           Odometry* odo,
            SpeedControl *scon) 
:SimpleWalker(leftWheel,rightWheel,odo,scon),
mOdo(odo),
mState(UNDEFINED)
{
    //msg_f("Turn create",1);
    mPID = new HPID();

    mTargetAngle=0.0f;
    mCurrentAngle=0.0f;
    mTurnPow = 0.0f;
    mFwdPow = 0.0f;
    mLowPWM = 1.0f;
}

Turn::~Turn()
{
    delete mPID;
}

/* 未定義状態
 * 初期処理に移行
*/
void Turn::execUndefined() 
{
    mState = INIT;
}

/* 初期処理
 * 旋回に必要な値を指定後に旋回処理に移行
*/
void Turn::initial()
{

    mState = TURNING;
}

/* 旋回処理
 * 指定した角度で終了
*/
void Turn::execTurning()
{
    /* //PID制御していた時のコード
    if(mFwdPow==0){
        //ブロックを持たない場合
        //float angle = mOdo->getAngle()*180/M_PI;  //deg
        float angle = mPoller->getGyroAngle();  //deg
        float turn = mPID->getOperation(angle);
        if(turn < mLowPWM &&
           turn > 0 ){
            setTurn(mLowPWM);
        }else if(turn > -mLowPWM &&
                 turn < 0 ){
            setTurn(-mLowPWM);
        }else{
            setTurn(turn);
        }
    }else{
        //ブロックを持つ場合
    }*/
    
    SimpleWalker::setCommand(mFwdPow,mTurnPow);           //
    SimpleWalker::run();

    /*  
    char buf[256];
    sprintf(buf,"turn:get:%f",mOdo->getGyroAngle());
    msg_f(buf,11);
    */
}

/* 終了処理
 * モーターを停止
*/
void Turn::end()
{
    SimpleWalker::setCommand(0,0);
    SimpleWalker::run();
    //msg_f("TURN:END",2);
}

/* 旋回処理を管理
*/
void Turn::run()
{
    //msg_f("Turn run",2);
    
    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case INIT:
            initial();
            break;
        case TURNING:
            execTurning();
            break;
        case END:
            end();
            break;
    }
}

/* 旋回時に使う前進値を指定
 * 通常は０、ブロックを運ぶときに増加
 * 
 * f                :float      :前進値
*/
void Turn::setFwd(float f)
{
    mFwdPow = f;
}

/* 旋回時に使う旋回値を指定
 * 通常はPID制御、ブロックを運ぶときは固定値
 * 
 * t                :float      :旋回値
*/
void Turn::setTurn(float t)
{
    mTurnPow = t;
}

/* 旋回の方向と角度を指定
 * 旋回前に指定する必要がある
 * 
 * direction        : 1         :右方向
 *                  :-1         :左方向
 * angle            :float      :角度（°）
*/
void Turn::setParam(int direction, float angle)
{
    setParam((float)direction * angle);

}

/* 旋回の角度を指定
 * 旋回前に指定する必要がある
 * 
 * angle            :float      :角度（°）
*/
void Turn::setParam(float angle)
{
    //mCurrentAngle = mOdo->getAngle();
    //mTargetAngle = mCurrentAngle * direction * angle *M_PI/180.0f;
    setCurrentAngle( mOdo->getAngle()*180/M_PI );
    //setTargetAngle( mCurrentAngle + (angle * M_PI/180.0f) );
    setTargetAngle( mCurrentAngle + angle );
    //setTurnPID(mTargetAngle);

    //char buf[256];
    //sprintf(buf,"turn:current:%f",mCurrentAngle);
    //msg_f(buf,8);
    //sprintf(buf,"turn:target:%f",mTargetAngle);
    //msg_f(buf,9);
    //sprintf(buf,"turn:angle:%f",angle);
    //msg_f(buf,10);
}

/* PID計算結果の上限値を指定
 * PIDを計算した結果が上限値を超えた場合には上限値が適用される
 * 
 * limit            :double     :上限値
*/
void Turn::setLimit(double limit) 
{
    mPID->setLimit(limit);
}

/* 旋回に使用するPID値などを指定
 * 
 * angle            :float      :旋回する角度
*/
void Turn::setTurnPID(float angle)
{
    mPID->setTarget(angle);
    mPID->resetParam();

    //180°旋回用(rad)
    //mPID->setKp(150.0f);
    //mPID->setKi(40.0f);
    //mPID->setKd(50.0f);

    //180°旋回用(°)
    mPID->setKp(2.5f);
    mPID->setKi(0.1f);
    mPID->setKd(0.0f);
    //mPID->setKi(0.5f);
    //mPID->setKd(0.8f);

    //mPID->setKi(20.0f);
    //mPID->setKi(60.0f);
    //mPID->setKd(2.5f);
    //mPID->setLimit(15);
}

/* 旋回時PWMの下限値を指定
 * 
 * pwm              :float      :下限値
*/
void Turn::setLowPWM(float pwm) 
{
    mLowPWM = pwm;
}

/* 状態を未定義に変更
*/
void Turn::setStateUndefined()
{
    mState = UNDEFINED;
}

/* 現在の角度を指定
 * 
 * angle            :float      :角度（rad）
*/
void Turn::setCurrentAngle(float angle)
{
    mCurrentAngle = angle;
}

/* 旋回後の角度を指定
 * 
 * angle            :float      :角度（rad）
*/
void Turn::setTargetAngle(float angle)
{
    mTargetAngle = angle;

}

/* 指定した旋回後の角度を修正
 * 
 * direction        : 1         :右方向
 *                  :-1         :左方向
 * angle            :float      :角度（rad）
*/
void Turn::addTargetAngle(int direction, float angle)
{
    //mTargetAngle+=angle;
    setTargetAngle( mTargetAngle += (direction * angle) );
}

/* 旋回が終了したかどうかを返す
 * 
 * return           :1          :完了
 *                  :0          :未完了
*/
int Turn::isEnd() 
{
    return mState==END;
}

/* 旋回値を返す
 * 
 * return           :float      :旋回値
*/
float Turn::getTurn() 
{
    return mTurnPow;
}
