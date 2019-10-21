/******************************************************************************
 *  SimpleWalker.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class SimpleWalker
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "SimpleWalker.h"
#include "Const.h"

#include "util.h"
// 定数宣言
/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
SimpleWalker::SimpleWalker(ev3api::Motor& leftWheel,
                        ev3api::Motor& rightWheel,
                        Odometry *odo,
                        SpeedControl *scon
                       ):
        mLeftWheel(leftWheel),
        mRightWheel(rightWheel),
        mOdo(odo),
        mSpeedControl(scon),
        mForward(0),
        mTurn(0),
        mBreake_flag(false),
        mMode_flag(false)
{
    mSpeedPid = new HPID(0.08);
    mSpeedPid->debug = false;
    mLPF = new HLowPassFilter(2);
    mLPF->setRate(0.995);
    battery = ev3_battery_voltage_mV();
}

/**
 * バランス走行する
 */
void SimpleWalker::run() {
  //   msg_f("SimpleWalker",2);
  //  int16_t angle = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
  char buf[256];
 static double speed=0;
 /*if(speed<mOdo->getVelocity()) {
    speed = mOdo->getVelocity();
     sprintf(buf,"max speed %f",speed);
     msg_f(buf,5);
 }*/
 /*
  static int cnt=0;
  if(!mBreake_flag && cnt++==20 && mMode_flag) { // 80ms毎に速度制御
    double op = mSpeedPid->getOperation(mOdo->getVelocity());
   // if (mOdo->getAccel()<10 && mOdo->getAccel()>-10) 
        mForward += (int)op; 
    if(mForward>75) mForward=75;
    if(mForward<-75) mForward=-75;
   
    cnt=0;  
    //char buf[256];
    //sprintf(buf,"op%3.1f fwd%2.1f t%3d",op,mForward,mTurn);
    //msg_f(buf,12);

  }*/
  mForward = mSpeedControl->getPwm();

  double pwm_l = mForward + mTurn;      // <2>
  double pwm_r = mForward - mTurn;      // <2>
    //char buf[256];
    //sprintf(buf,"SW: %f : %f",pwm_l,pwm_r);
    //msg_f(buf,12);


    //pwm_l = pwm_l*8220.0/battery;
    //pwm_r = pwm_r*8220.0/battery;
   static const int MAXPWM=80;
/*
    if(pwm_l>MAXPWM) {
        pwm_r = (int)((float)MAXPWM*pwm_r/pwm_l);
        pwm_l=MAXPWM;
    }
    if(pwm_l<-MAXPWM) {
        pwm_r = (int)((float)-MAXPWM*pwm_r/pwm_l);
        pwm_l=-MAXPWM;
    }

    if(pwm_r>MAXPWM) {
        pwm_l = (int)((float)MAXPWM*pwm_l/pwm_r);
        pwm_r=MAXPWM;
    }
    if(pwm_r<-MAXPWM) {
        pwm_l = (int)((float)-MAXPWM*pwm_l/pwm_r);
        pwm_r=-MAXPWM;
    }*/

    if(pwm_r>100) pwm_r=100;
    if(pwm_l>100) pwm_l=100;
    if(pwm_r<-100) pwm_r=-100;
    if(pwm_l<-100) pwm_l=-100;

    battery = ev3_battery_voltage_mV();
   // mLPF->addValue(battery);
   // battery = mLPF->getFillteredValue();

    double volt_adj = adjustBattery(BASE_VOLT,battery);
    //volt_adj = 1.0;
    static int prev_l = 0;
    static int prev_r = 0;
   int left = (int)(pwm_l*volt_adj);
   int right = (int)(pwm_r*volt_adj);
    mLeftWheel.setPWM(left);
    mRightWheel.setPWM(right);

    mOdo->setPwm(prev_l,prev_r);

    prev_l = left;
    prev_r = right;



}

/**
 * バランス走行に必要なものをリセットする
 */
void SimpleWalker::init() {
  //  int offset = mGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    int offset = 0;
    // モータエンコーダをリセットする
    mLeftWheel.reset();
    mRightWheel.reset();
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void SimpleWalker::setCommand(int forward, int turn) {
    mTargetSpeed = forward;
    mForward = forward;
    mSpeedControl->resetParam();
    mSpeedControl->setTargetSpeed(forward);
    mTurn    = turn;
    //mMode_flag = false;
    mSpeedControl->setMode(false);

}

void SimpleWalker::setCommandV(double forward, int turn)
{

    mTargetSpeed = forward;
    mSpeedControl->setTargetSpeed(forward);

    //mBreake_flag=false;
    mSpeedControl->setBreak(false);
    
    if (forward==0) {
        //mBreake_flag=true;
        mSpeedControl->setBreak(true);
        mForward=0;
    }
    mTurn    = turn;
    //mMode_flag = true;
    mSpeedControl->setMode(true);
}

void SimpleWalker::resetParam()
{
    //mSpeedPid->resetParam();   
    mSpeedControl->resetParam();
}

double SimpleWalker::adjustBattery(int base,int volt)
{
    double gain = 0.001089;
    double offset = 0.625;

    double adj = 1.0;

    if(ADJUST_BATTERY) {
        double base_param = gain*base-offset;
        double current = gain*volt - offset;

        adj = base_param/current;
        if(adj>1.2) adj=1.2;
        if(adj<0.8) adj=0.8;
    }

    return adj;
}


