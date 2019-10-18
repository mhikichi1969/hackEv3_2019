/******************************************************************************
 *  LineTracer.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/26
 *  Implementation of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "LineTracer.h"
#include <math.h>

#include "util.h"


const int LineTracer::LINE    = 25;   
const int LineTracer::LEFTEDGE = true;
const int LineTracer::RIGHTEDGE = false;

/**
 * コンストラクタ
 * @param lineMonitor     ライン判定
 * @param balancingWalker 倒立走行
 */
LineTracer::LineTracer(ev3api::Motor& leftWheel,
                        ev3api::Motor& rightWheel,    
                       HPolling* poller,
                       Odometry* odo,
                        SpeedControl *scon
                    ) 
    : SimpleWalker(leftWheel,rightWheel,odo,scon),
    mPoller(poller),
    mState(UNDEFINED),
      mIsInitialized(false),
      mSpeedPid(),
      mSpeed(0),
      mLeftEdge(false),
      mBackMode(false),
      mLimit(100),
      mCurve(0),
      mBias(0),
      mAuto(false) {

          mPid = new HPID();
          mPid->debug = false;
          mPid->setTarget(mTarget);  //ラインの輝度
          mPid->setKp(mPFactor); //0.376
          mPid->setKi(mIFactor);
          mPid->setKd(mDFactor);
          mPid->setLimit(mLimit);

          mDirectionPid = new HPID();
          mDirectionPid->setLimit(mLimit);
          
         // EV3-way
          //mPid.setKp(0.5); 
         // mPid.setKi(2.52);
         // mPid.setKd(0.00147);

          mSpeedPid.setTarget(mTargetSpeed);
          mSpeedPid.setKp(0.1); 

          mTurnZeroCnt=0;
          mTurn=0;

}

void LineTracer::init() {
    //mPoller->setColorSensorMode(HPolling::REFLECT);
    mState = RUNNING;
}

 
void LineTracer::run()
{
    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case RUNNING:
            running();
            break;
        case CHANGING:
            changeEdgeMode();
            break;
    }

}

void LineTracer::execUndefined() 
{
    init();
    mState = RUNNING;
}


/**
 * ライントレースする
 */
void LineTracer::running() {
    /*if (mIsInitialized == false) {
        mBalancingWalker->init();
        mIsInitialized = true;
    }*/

  //  msg_f("lineTracer run",2);

    double  brightness;
    double direction;
    brightness=mPoller->getBrightnessRate();
    if(mAuto) {
            direction = mOdo->getWheelCountDiffFromVelocity(mCurve);
    } else {
        if(mSpeed>0 ) 
            direction = mOdo->getWheelCountDiffFromRecord(mCurve);
        else 
            direction = mOdo->getWheelCountDiffFromRecordReverse(mCurve);     
    }
    mTurn = calcTurn(brightness,direction);

   // mTurn += -mBias;

    //setCommand((int)mTargetSpeed, (int)mTurn);

    /*char buf[256];
    sprintf(buf,"LT %d,%3.1f",mTargetSpeed,mTurn);
    msg_f(buf,2);*/

    setCommandV((int)mTargetSpeed, (int)mTurn);

    SimpleWalker::run();
}

float LineTracer::calcTurn(float val1,float val2,float val3) {
    float val1_turn =  mPid->getOperation(val1);
    if(mLeftEdge) val1_turn = -val1_turn;
    if(mTargetSpeed<0) val1_turn = -val1_turn;

    float turn =  val1_turn + mAnglePid.getOperation(val2);
    turn += mDirectionPid->getOperation(val3);

    if(turn>100.0) turn = 100.0;
    if(turn<-100.0) turn = -100.0;
    return turn;
 
}
    
float LineTracer::calcTurn(float val1,float val2) {

    // 速度制御する場合にはPIDパラメータを調整
    //double bai = fabs(mOdo->getVelocity()/mTargetSpeed);      //SpeedSection用
    double bai = 1.0;   //CompositeSection用？
    if(mSpeedControl->getCurrentSpeed()<12) {
       // ev3_speaker_play_tone(NOTE_F4,10);
        bai=0.6;
    }
    //if (bai>1.0) bai=1.0;
    mPid->setKp(mPFactor*bai); //0.376
    mPid->setKi(mIFactor*bai);
    mPid->setKd(mDFactor*bai);
   // mPid->debug=true;

    float val1_turn =  mPid->getOperation(val1);
    if(mLeftEdge) val1_turn = -val1_turn;
    if(mTargetSpeed<0) val1_turn = -val1_turn;

    battery = ev3_battery_voltage_mV();
    mLPF->addValue(battery);
    battery = mLPF->getFillteredValue();
    double adj = adjustBattery(BASE_VOLT,battery);
    val1_turn *= adj;

   // float turn =  val1_turn + mAnglePid.getOperation(val2);

  //  addBias(mAnglePid.getOperation(val2));
    //setBias(-mTargetSpeed*(1-mCurve)/(1+mCurve)*mAngleKp);
    setBias(-mForward*(1-mCurve)/(1+mCurve)*mAngleKp);
    float turn =  val1_turn+mBias;


   /* char buf[256];
   sprintf(buf,"turn:%f %f",val1_turn,turn);
    msg_f(buf,13);
    */

    if((int)turn==0) 
        mTurnZeroCnt++;
    else 
        mTurnZeroCnt=0;


       
    
   double t_limit = SimpleWalker::mForward>40?SimpleWalker::mForward*0.9:SimpleWalker::mForward*1.1;
    t_limit = SimpleWalker::mForward>10?t_limit:SimpleWalker::mForward*1.4;
    

   if(turn>t_limit) turn = t_limit;
    if(turn<-t_limit) turn = -t_limit;
   // if(turn>50) turn = 50;
   // if(turn<-50) turn = -50;

    //double volt_adj = 8220.0/mPoller->getVoltage();
    return turn;
}

void LineTracer::setParam(float speed,float target,float kp, float ki, float kd) 
{

    //ev3_speaker_play_tone(NOTE_C4,100);
    mTargetSpeed = speed;
    mTarget= target;
    mPFactor = kp;
    mIFactor = ki;
    mDFactor = kd;

    mPid->setTarget(mTarget);
   // mPid.setLimit(speed);
    mPid->setKp(mPFactor); //0.376
    mPid->setKi(mIFactor);
    mPid->setKd(mDFactor);

  /*  mSpeedPid.setTarget(mTargetSpeed);
    mSpeedPid.setKp(0.01); 
    mSpeedPid.setLimit(1);*/

    mCurve = 1.0;
    mAnglePid.setTarget(0);
    mAnglePid.setKp(0);
    mAnglePid.setKi(0);
    mAnglePid.setKd(0);

    //mAnglePid.setLimit(100-mTarget);

}

void LineTracer::setParam(float speed,float target,float kp, float ki, float kd,
                        float angleTarget,float angleKp) 
{

    mTargetSpeed = speed;
    mTarget= target;
    mPFactor = kp;
    mIFactor = ki;
    mDFactor = kd;

    mAngleKp = angleKp;

    mPid->setTarget(mTarget);
    double t = mPid->getTarget();
    char buf[256];
 //   sprintf(buf,"setparam %l.1f,%l.1f",mTarget,t);
 //   msg_f(buf,12);

   // mPid.setLimit(100-mTarget);
    mPid->setKp(mPFactor); //0.376
    mPid->setKi(mIFactor);
    mPid->setKd(mDFactor);
    //mPid->setLimit(SimpleWalker::mForward*1.0);

   /* mSpeedPid.setTarget(mTargetSpeed);
    mSpeedPid.setKp(0.01); */

    mCurve = angleTarget;
    mAnglePid.setTarget(0);
    mAnglePid.setKp(angleKp);
    mAnglePid.setKi(0);
    mAnglePid.setKd(0);

    setParamDirection(0,0,0,0);
   // mAnglePid.setLimit(100-mTarget);
}


void LineTracer::setParam(float speed,
                        float target,float kp, float ki, float kd,
                        float angleTarget,float angleKp, float angleKi, float angleKd) 
{
    mTargetSpeed = speed;
    mTarget= target;
    mPFactor = kp;
    mIFactor = ki;
    mDFactor = kd;


    mPid->setTarget(mTarget);
   // mPid.setLimit(100-mTarget);
    mPid->setKp(mPFactor); //0.376
    mPid->setKi(mIFactor);
    mPid->setKd(mDFactor);

    mSpeedPid.setTarget(mTargetSpeed);
    mSpeedPid.setKp(0.01); 

    mCurve = angleTarget;
    mAnglePid.setTarget(0);
    mAnglePid.setKp(angleKp);
    mAnglePid.setKi(angleKi);
    mAnglePid.setKd(angleKd);
    setParamDirection(0,0,0,0);
    
   // mAnglePid.setLimit(100-mTarget);
}

void LineTracer::setParamDirection(float angleTarget,float angleKp, float angleKi, float angleKd) 
{
    mDirectionPid->setTarget(angleTarget);
    mDirectionPid->setKp(angleKp);
    mDirectionPid->setKi(angleKi);
    mDirectionPid->setKd(angleKd);

}

void LineTracer::setEdgeMode(bool isLeftEdge)
{
    mLeftEdge = isLeftEdge;
}

bool LineTracer::getEdgeMode()
{
    return mLeftEdge;
}

void LineTracer::changeEdge()
{
    mLeftEdge = !mLeftEdge;
}

void LineTracer::changeEdgeMode()
{
    mState=CHANGING;
    mPid->setTarget(-1.0f); 
    running();

    if(mPoller->getBrightnessRate()<-0.3f) {
        mPid->setTarget(mTarget); 
        setEdgeMode(!mLeftEdge);
        mState=RUNNING;
    }
}

void LineTracer::setLimit(float limit)
{
    mLimit=limit;
    mPid->setLimit(limit);
}

float LineTracer::getTurn() 
{
    return mTurn;
}

int LineTracer::getTurnZeroCnt() 
{
    return mTurnZeroCnt;
}

void LineTracer::resetSpeed(float speed)
{
    mSpeed=speed;
}

bool LineTracer::isLeftEdge()
{
    return mLeftEdge;
}

float LineTracer::getIntegral()
{
    return mPid->getIntegral();
}

void LineTracer::resetParam()
{
    SimpleWalker::resetParam();
    mAnglePid.resetParam();
    mDirectionPid->resetParam();
    //mOdo->reset();
    //mPoller->resetGyroAngle();
}
void LineTracer::resetLinePid()
{
    mPid->resetParam();
}

void LineTracer::setBias(double curve)
{
    //mBias = -mTargetSpeed*(1-curve)/(1+curve)*1.9;
     mBias = curve;
}

void LineTracer::addBias(double add)
{
    mBias += add;
}

double LineTracer::adjustBattery(int base,int volt)
{

    double adj = 1.0;

    if(ADJUST_BATTERY2 && mTargetSpeed>=50) {
        adj = (ADJUST_PARAM-1.0)*(volt-8500)/500.0+1.0;
    }

    return adj;
}
