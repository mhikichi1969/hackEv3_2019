/******************************************************************************
 *  Starter.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Starter
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "Starter.h"

/**
 * コンストラクタ
 * @param touchSensor タッチセンサ
 */
Starter::Starter(const ev3api::TouchSensor& touchSensor)
    : mTouchSensor(touchSensor),
    mState(UNDEFINED),
    mPushed(false) {
}

/**
 * 押下中か否か
 * @retval true  押下している
 * @retval false 押下していない
 */
bool Starter::isPushed()  {
    static int cnt=0;
    static int relese_cnt=0;
    if(mTouchSensor.isPressed()) {
        cnt++;
        relese_cnt=0;
    }else {
        cnt=0;
        relese_cnt++;        
    }
    if( cnt>25 ) {
        mPushed = true;
    } else if(relese_cnt>15) {
        mPushed = false;
    }
    return mPushed;
}

void Starter::run() {
    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case INITIAL:
            execInitial();
            break;
        case WAIT:
            execWait();
            break;
        case RELEASE:
            execRelease();
            break;
        case TOUCHED:
            break;
    }
}

void Starter::execUndefined()
{
    mState = INITIAL;
}

void Starter::execInitial()
{
    mState = WAIT;
}

void Starter::execWait()
{
    if(isPushed())  
        mState = RELEASE;
}

void Starter::execRelease()
{
    if(!isPushed()) {
        mState = TOUCHED;
    }
}

bool Starter::isTouched()
{
    return mState==TOUCHED;
}
