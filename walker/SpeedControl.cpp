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
    double bai =0.7;
    //float bai =0.3;
    //float bai =speed/60.0;
    if(fabs(mTargetSpeed)<50) bai=0.45;
    if(fabs(mTargetSpeed)<36) bai=0.3;
   // if(fabs(mTargetSpeed)<31) bai=0.25;

    if(mTargetSpeed!=speed) {
        mPid->resetParam();
        if(mTargetSpeed*speed<0) {
            bai =0.9;
        }
        if(fabs(mTargetSpeed)>fabs(speed)) {
            bai =1.2;    
        }
    }

    mTargetSpeed = speed;

    mPid->setTarget(speed);

    mPid->setKp(0.65*bai);
    mPid->setKi(0.3*bai);
        //mPid->setKd(0.03*bai);
    mPid->setKd(0.02*bai);
    mPid->setLimit(10*bai);    
    //mPid->setLimit(1);    

}

int SpeedControl::getPwm()
{
    static int cnt=0;
    // 直接制御なら
    if(!mMode_flag) {
	    //ev3_speaker_play_tone(NOTE_F4,50);
        mForward = mTargetSpeed;
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
    
    /*int battery = ev3_battery_voltage_mV();
    double adj = adjustBattery(9000,battery);*/
    int maxFwd = 75;
    
    if(mForward>maxFwd) {
        ev3_speaker_play_tone(NOTE_F4,50);
        mForward=maxFwd;
    }

    if(mForward<-maxFwd) {
        ev3_speaker_play_tone(NOTE_F4,50);
       mForward=-maxFwd;
    }
   
    cnt=0;  
    char buf[256];
    sprintf(buf,"op%3.1f fwd%2.1d v:%2.1f",op,mForward,mOdo->getVelocity());
   // msg_f(buf,12);


    }
    return mForward;
}

void SpeedControl::resetParam()
{
    mForward = 0;
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

double SpeedControl::adjustBattery(int base,int volt)
{
    double gain = 0.001089;
    double offset = 0.625;

    double adj = 1.0;

    if(true) {
        double base_param = gain*base-offset;
        double current = gain*volt - offset;

        adj = base_param/current;
        if(adj>1.2) adj=1.2;
        if(adj<0.8) adj=0.8;
    }

    return adj;
}
