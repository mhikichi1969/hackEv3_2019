#ifndef _HARMCONTROL_H_
#define _HARMCONTROL_H_

#include "Motor.h"
#include "HBTtask.h"
#include "HPID.h"
#include "util.h"

//using namespace ev3api;

class ArmControl {
public:
    ArmControl(ev3api::Motor& arm,HBTtask *bt);
    ~ArmControl();

    void run();
    void lock();

    void decAngle();
    void incAngle();
    void dec10Angle();
    void inc10Angle();
    void throwBlock();
    void setAngle(double ang);
    void setLimit(double limit);
    void setPID(float t, float p, float i, float d);
    void setPwm(double pwm);

    void execUndefined();
    void execInit();
    void execPIDRun();
    void execPowRun();

    void changeStateRun();

    double getAngle();

private:
    enum State {
        EXEC_UDF,
        EXEC_INI,
        EXEC_PID,
        EXEC_POW
    };

    ev3api::Motor& mArm;
    HBTtask *mBt;
    //HPID* mPID;                 //左右モーターの回転数の差を制御

    State mState;

    double angle;
    
    double kp;
    
    double bAngle;
};

#endif