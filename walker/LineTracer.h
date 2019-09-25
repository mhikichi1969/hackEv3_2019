/******************************************************************************
 *  LineTracer.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Class LineTracer
 *  Author: Kazuhiro Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#ifndef EV3_APP_LINETRACER_H_
#define EV3_APP_LINETRACER_H_

#include "SimpleWalker.h"
#include "HPID.h"
#include "HPolling.h"
#include "Odometry.h"
#include "SpeedControl.h"
#include "Const.h"

class LineTracer : public SimpleWalker {
public:
    LineTracer(ev3api::Motor& leftWheel,
                ev3api::Motor& rightWheel, 
               HPolling* polling,
               Odometry* odo,
                SpeedControl *scon);
    void init();
    void run();
    void running();

    void execUndefined();
    void modeChange();
    void setParam(float speed,float target,float kp, float ki, float kd);
    void setParam(float speed,float target,float kp, float ki, float kd,float angleTarget,float angleKp);
    void setParam(float speed,
                        float target,float kp, float ki, float kd,
                        float angleTarget,float angleKp, float angleKi, float angleKd);
    void setParamDirection(float angleTarget,float angleKp, float angleKi, float angleKd);
                        
    void setEdgeMode(bool isLeftEdge);
    bool getEdgeMode();
    void changeEdgeMode();
    void setLimit(float limit);

    float getTurn();
    int getTurnZeroCnt();
    void resetSpeed(float speed);

    bool isLeftEdge();
    float getIntegral();

    void resetParam();
    void changeEdge();

    void setBias(double curve);
    void addBias(double add);
    double adjustBattery(int base,int volt);

    static const int LINE;
    static const int LEFTEDGE;
    static const int RIGHTEDGE;


private:
    enum State {
        UNDEFINED,
        RUNNING,
        CHANGING
    };
    HPolling *mPoller;
 //   Odometry *mOdo;

    State mState;

    bool mIsInitialized;

    int calcDirection(bool isOnLine);
    float calcTurn(float val1,float val2);
    float calcTurn(float val1,float val2,float val3);
    
    HPID mSpeedPid;
    float mSpeed;
    HPID *mPid;
    HPID mAnglePid;
    HPID *mDirectionPid;


    int mTargetSpeed;
    double mTarget;
    float mPFactor;
    float mIFactor;
    float mDFactor;

    double mAngleKp;

    bool mLeftEdge;
    bool mBackMode;

    float mLimit;
    float mCurve;

    //float mTurn;
    int mTurnZeroCnt;

    double mBias;

    bool mAuto;

    
};

#endif  // EV3_APP_LINETRACER_H_
