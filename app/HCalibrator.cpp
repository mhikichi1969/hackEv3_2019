/******************************************************************************
 *  Starter.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Class Starter
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *****************************************************************************/

#include "HCalibrator.h"


HCalibrator::HCalibrator(HPolling *poller,
                        ArmControl *arm
                        //HPuzzle *puzzle
                        ):
    mPoller(poller),
    mArmControl(arm),
   // mPuzzle(puzzle),
    mState(UNDEFINED)
{
    code=0; // ブロック位置決定
    white=0; // 白キャリブレーション
    color=0; // カラーセンサーキャリブレーション
    black=0; // 黒キャリブレーション
    
}

void HCalibrator::run(){

    switch(mState) {
        case UNDEFINED:
            execUndefined();
            break;
        case INIT:
            execInit();
            break;
        case PRESSED:
            execPressed();
            break;
        case CHANGE:
            execChange();
            break;
        case END:
            break;
    }

    
}


void HCalibrator::execUndefined()
{
    dial = mPoller->getLeftCount();
    mState = INIT;
}



void HCalibrator::execInit()
{
    if(dial-mPoller->getLeftCount()>30) {
        digit++;
        digit%=10;
        dial = mPoller->getLeftCount();
        ev3_speaker_play_tone(NOTE_C4,5);

    }
    if(dial-mPoller->getLeftCount()<-30) {
        digit--;
        if(digit<0) digit=9;
       dial = mPoller->getLeftCount();
        ev3_speaker_play_tone(NOTE_C4,5);

    }

    if(ev3_button_is_pressed(LEFT_BUTTON)) {
        mPressNo=LEFT_BUTTON;
        mState=PRESSED;
    }
    if(ev3_button_is_pressed(RIGHT_BUTTON )) {
        mPressNo=RIGHT_BUTTON;
        mState=PRESSED;
    }
    if(ev3_button_is_pressed(UP_BUTTON)) {
        mPressNo=UP_BUTTON;
        mState=PRESSED;
    }
    if(ev3_button_is_pressed(DOWN_BUTTON )) {
        mPressNo=DOWN_BUTTON;
        mState=PRESSED;
    }
    if(ev3_button_is_pressed(ENTER_BUTTON )) {
        mPressNo=ENTER_BUTTON;
        mState=PRESSED;
    }
    if(ev3_button_is_pressed(BACK_BUTTON )) {
        mPressNo=BACK_BUTTON;
        mState=PRESSED;
    }

    static char buf[256];
    static char buf2[256];
    static char buf3[256];
    static char buf4[256];
    static char buf5[256];

    static int col_cnt[8]={0,0,0,0,0,0,0,0};

    if(code>=0 && code<=2) {
        char course_str[] = {'L','R'};
        blockCode[code] = digit;
        int course_idx = blockCode[0]%2;
        sprintf(buf3,"CODE: %c %d %d",course_str[course_idx],blockCode[1],blockCode[2]);
    } else if(color==0) {
        mPoller->setColorSensorMode(HPolling::COLOR);

        int id = (int)mPoller->getColorID();
        if(id==7) id=4; // 茶色を黄色にカウント
        col_cnt[id]++;
        sprintf(buf,"NONE:%d K:%d B:%d G:%d Y:%d R:%d",
            col_cnt[0],
            col_cnt[1],
            col_cnt[2],
            col_cnt[3],
            col_cnt[4],
            col_cnt[5]);  

        rgb_raw_t rgb;
        rgb = mPoller->getRGB();
        sprintf(buf4," R:%d,G:%d,B:%d ", rgb.r,rgb.g,rgb.b);

    } else if(white==0) {
        mPoller->setColorSensorMode(HPolling::REFLECT);

        sprintf(buf,"W:%d",mPoller->getBrightness());
    } else if(white==1) {
        mPoller->setColorSensorMode(HPolling::COLOR);
        rgb_raw_t rgb = mPoller->getRGB();

        sprintf(buf,"W:%d WC:%d,%d,%d",mPoller->getWhiteLevel(),
                                                rgb.r,
                                                rgb.g,
                                                rgb.b);
        sprintf(buf2," ");


    }else if(black==0) {
         mPoller->setColorSensorMode(HPolling::REFLECT);

        sprintf(buf,"W:%d   WC:%d,%d,%d ",mPoller->getWhiteLevel(),
                                                mPoller->getWhiteLevelC().r,
                                                mPoller->getWhiteLevelC().g,
                                               mPoller->getWhiteLevelC().b);
        sprintf(buf2,"B:%d", mPoller->getBrightness());
    } else if(black==1) {
        mPoller->setColorSensorMode(HPolling::COLOR);
        rgb_raw_t rgb = mPoller->getRGB();

        sprintf(buf,"W:%d   WC:%d,%d,%d",mPoller->getWhiteLevel(),
                                                mPoller->getWhiteLevelC().r,
                                                mPoller->getWhiteLevelC().g,
                                               mPoller->getWhiteLevelC().b);
        sprintf(buf2,"B:%d BC:%d,%d,%d", mPoller->getBlackLevel(),
                                       rgb.r, rgb.g, rgb.b );

        rgb = mPoller->getRGB();
        hsv_t hsv;
        mPoller->getHSV(hsv);
        sprintf(buf4," R:%d,G:%d,B:%d ", rgb.r,rgb.g,rgb.b );
        sprintf(buf5," H:%2.1f,S:%2.2f,V:%2.1f, No:%d" ,hsv.h,hsv.s,hsv.v,mPoller->getColorID());


    } else {
        mPoller->setColorSensorMode(HPolling::REFLECT);        
    }

    msg_f(buf,1);
    msg_f(buf2,2);
    msg_f(buf3,3);
    msg_f(buf4,4);
    msg_f(buf5,5);

}

void HCalibrator::execPressed()
{
    bool press=false;
    if(ev3_button_is_pressed(LEFT_BUTTON)) {
        press=true;
    }
    if(ev3_button_is_pressed(RIGHT_BUTTON)) {
        press=true;
    }
    if(ev3_button_is_pressed(UP_BUTTON)) {
        press=true;
    }
    if(ev3_button_is_pressed(DOWN_BUTTON)) {
        press=true;
    }
    if(ev3_button_is_pressed(ENTER_BUTTON)) {
        press=true;
    }
    if(ev3_button_is_pressed(BACK_BUTTON)) {
        press=true;
    }

    if(!press) {

        mState=CHANGE;
    }

}

void HCalibrator::execChange()
{
    if(!white) {
      //  ev3_speaker_play_tone(NOTE_C5,100);

        if(mPressNo==UP_BUTTON) {
            mArmControl->incAngle();
        }
        if(mPressNo==DOWN_BUTTON) {
            mArmControl->decAngle();
        }
        if(mPressNo==RIGHT_BUTTON) {
            mArmControl->inc10Angle();
        }
        if(mPressNo==LEFT_BUTTON) {
            mArmControl->dec10Angle();
        }
    }
    mState = INIT;
    if(mPressNo==ENTER_BUTTON) {
        if(code>=0 && code<=2) {
            digit=0;
            ev3_speaker_play_tone(NOTE_C4,20);
            code++;
        }else if(color==0) {
            //PuzzleStrategy *strategy = mPuzzle->getStrategy();
            //strategy->encodeBlock(blockCode[0]*10000+blockCode[1]*1000+blockCode[2]*100+blockCode[3]*10+blockCode[4]);

            color++;
          //  mArmControl->resetColorScanAngle();

            mState = INIT;           
        } else if(white==0) {
            white++;
            mPoller->setWhiteLevel();
           // mArmControl->resetAngle();
            mState = INIT;
        } else if(white==1) {
            white++;
            mPoller->setWhiteLevelC();
           // mArmControl->resetAngle();
            ev3_speaker_play_tone(NOTE_F4,80);

            mState = INIT;
        } else if(black==0) {
            black++;
            mPoller->setBlackLevel();
            mState = INIT;
        }else if(black==1) {
            black++;
            mPoller->setBlackLevelC();
             ev3_speaker_play_tone(NOTE_G4,80);

            mState = INIT;
        } 
        else {
          //  msg_f("reflect",8);
            mPoller->setColorSensorMode(HPolling::COLOR);
            ev3_speaker_play_tone(NOTE_B4,120);

            mState = END;
        }
    }

}

button_t HCalibrator::pressedButton()
{ 
    if(mState==CHANGE)
        return mPressNo;
    else 
        return (button_t)-1;
}

bool HCalibrator::isEnd()
{
    return mState==END;
}

int HCalibrator::getCodeNum(int idx)
{
    return blockCode[idx];
}