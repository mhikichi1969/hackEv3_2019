/******************************************************************************
 *  SimpleWaker.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class SimpleWaker
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_UNIT_SIMPLE_H_
#define EV3_UNIT_SIMPLE_H_

#include "Motor.h"
#include "HPolling.h"
#include "Odometry.h"
#include "HPID.h"
#include "SpeedControl.h"
#include "HLowPassFilter.h"

class SimpleWalker {
public:
    SimpleWalker(ev3api::Motor& leftWheel,
                ev3api::Motor& rightWheel,
                Odometry *odo,
                SpeedControl *scon
                );

    void init();
    virtual void run();
    void setCommand(int forward, int turn);
    void setCommandV(double forward, int turn);
    void resetParam();
    double adjustBattery(int base,int volt);

protected:


    double mForward;
    double mTargetSpeed;
    int mTurn;

    bool mBreake_flag;
    bool mMode_flag;//setComandVだとtrue、setComandだとfalse

    Odometry *mOdo;
    HPID *mSpeedPid;
    SpeedControl *mSpeedControl;
    HLowPassFilter *mLPF;
    int battery;

private:
    ev3api::Motor& mLeftWheel;
    ev3api::Motor& mRightWheel;

};

#endif  // EV3_UNIT_SimpleWaker_H_
