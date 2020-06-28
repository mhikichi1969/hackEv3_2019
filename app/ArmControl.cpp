#include "ArmControl.h"
#include "util.h"


ArmControl::ArmControl(ev3api::Motor& arm,HBTtask *bt):
mArm(arm),
mBt(bt)
{
    //mPID = new HPID();

    kp=1.5;       //runで使用していた値、diffに掛けて使用
    mArm.reset();
    mState = EXEC_UDF;
    bAngle = 999;

    angle = -39;  // -38
    
}

ArmControl::~ArmControl()
{
    //delete mPID;
}

void ArmControl::run()
{
    switch (mState)
    {
    case EXEC_UDF:
        execUndefined();
        break;
    
    case EXEC_INI:
        execInit();
        break;
    
    case EXEC_PID:
        execPIDRun();
        break;
    
    case EXEC_POW:
        execPowRun();
        break;
    
    default:
        break;
    }
}

void ArmControl::execUndefined()
{
    mState = EXEC_INI;
}

void ArmControl::execInit()
{
    mState = EXEC_PID;
}

void ArmControl::execPIDRun()
{

    //angle = mBt->arm;
    double diff = angle - mArm.getCount();
    float pwm = diff*kp;     
    if(pwm>50) pwm=50;
    if(pwm<-50) pwm=-50;
    mArm.setPWM(pwm);
    //mArm.stop();
    //ev3_motor_rotate(EV3_PORT_A ,0,100,true);
   /* char buf[256];
    sprintf(buf,"ARM %d",angle);
    msg_f(buf,11);*/

}

void ArmControl::execPowRun()
{
    //msg_f("Arm:POW_RUN",2);
}

void ArmControl::changeStateRun()
{
    if(mState == EXEC_PID){
        mState = EXEC_POW;
    }else if(mState == EXEC_POW){
        mState = EXEC_PID;
    }
}


void ArmControl::decAngle()
{
    angle--;
}
void ArmControl::incAngle()
{
    angle++;
}
void ArmControl::dec10Angle()
{
    angle-=10;
}
void ArmControl::inc10Angle()
{
    angle+=10;
}

void ArmControl::lock()
{
    mArm.setPWM(0);
}

void ArmControl::throwBlock()
{

    //char buf[256];
    //sprintf(buf,"ARM:angle %f",(double)angle);
    //msg_f(buf,7);
    
    double diff = angle - mArm.getCount();
    //float pwm = diff*0.7;
    float pwm = diff*kp;
    //float pwm = mPID->getOperation(diff);

    if(pwm>50) pwm=50;
    if(pwm<-50) pwm=-50;
    mArm.setPWM(pwm);
    //ev3_motor_rotate(EV3_PORT_A ,0,100,true);
    //char buf[256];
    //sprintf(buf,"ARM:pwm %f",(double)pwm);
    //msg_f(buf,8);

}

/* PID計算結果の上限値を指定
 * PIDを計算した結果が上限値を超えた場合には上限値が適用される
 * 
 * limit            :double     :上限値
*/
void ArmControl::setLimit(double limit) 
{
    //mPID->setLimit(limit);
}

/* 使用するPID値などを指定
 * 
*/
void ArmControl::setPID(float t, float p, float i, float d)
{
    //mPID->setTarget(0);
    //mPID->setTarget(t);
    //mPID->resetParam();

    //
    //mPID->setKp(20.0f);
    //mPID->setKi(5.0f);
    //mPID->setKd(15.0f);
    //mPID->setKp(p);
    //mPID->setKi(i);
    //mPID->setKd(d);

    //mPID->setLimit(15);

}


/* アーム回転力を指定
 * 
*/
void ArmControl::setPwm(double pwm) 
{
    mArm.setPWM(pwm);
}

/* アーム目標角度を相対的に指定
 * 
*/
void ArmControl::setAngle(double ang)
{
    if(bAngle==999){
        bAngle = angle;
    }
    angle = bAngle + ang;
}

/* 目標角度を取得
 * 
*/
double ArmControl::getAngle()
{
    return angle;
}

