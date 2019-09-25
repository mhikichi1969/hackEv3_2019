#include "SpeedControl.h"

SpeedControl::SpeedControl(Odometry *odo):
    mOdo(odo),
    mForward(0),
    mCurrentSpeed(0.0),
    mMode_flag(true),
    mBreake_flag(false)
{
    mPid = new HPID(0.08);
    mPid->debug = false;

}

void SpeedControl::setTargetSpeed(double speed)
{
    static double prev_speed=0;
    double bai =1.0;
    //float bai =0.3;
    //float bai =speed/60.0;
    if(speed<50) bai=0.5;
    if(speed<30) bai=0.3;


    mTargetSpeed = speed;
    mPid->resetParam();

    mPid->setTarget(speed);

    mPid->setKp(0.6*bai);
    mPid->setKi(0.05*bai);
        //mPid->setKd(0.03*bai);
    mPid->setKd(0.02*bai);
    mPid->setLimit(8*bai);    
    //mPid->setLimit(1);    

}

int SpeedControl::getPwm()
{
    static int cnt=0;
    // 直接制御なら
    if(!mMode_flag) {
	    //ev3_speaker_play_tone(NOTE_F4,50);
        return mTargetSpeed;
    }
    //停止モード
    if(mBreake_flag) {
        mForward=0;
        return 0;
    }
  if(cnt++==20) { // 80ms毎に速度制御
    mCurrentSpeed = mOdo->getVelocity();
    double op = mPid->getOperation(mCurrentSpeed);
   // if (mOdo->getAccel()<10 && mOdo->getAccel()>-10) 
        mForward += (int)op; 
    if(mForward>75) mForward=75;
    if(mForward<-75) mForward=-75;
   
    cnt=0;  
    char buf[256];
    sprintf(buf,"op%3.1f fwd%2.1d v:%2.1f",op,mForward,mOdo->getVelocity());
   // msg_f(buf,12);


    }
    return mForward;
}

void SpeedControl::resetParam()
{
    mForward=0;
    mPid->resetParam();
}

void SpeedControl::setMode(bool mode)
{
    mMode_flag=mode;
}
void SpeedControl::setBreak(bool brk)
{
    mBreake_flag=brk;
}

double SpeedControl::getCurrentFwd()
{
    return mForward;
}

double SpeedControl::getCurrentSpeed()
{
    return mCurrentSpeed;
}