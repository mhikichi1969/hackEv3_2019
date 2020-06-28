#ifndef HCALIBRATOR_
#define HCALIBRATOR_

#include "ev3api.h"
#include "HPolling.h"
#include "ArmControl.h"
//#include "HPuzzle.h"
//#include "PuzzleStrategy.h"


class HCalibrator
{
    public:
        HCalibrator(HPolling *poller,
                    ArmControl *arm
                    //HPuzzle *puzzle
                    );

        void run();
        void execUndefined();
        void execInit();
        void execPressed();
        void execChange();
        button_t pressedButton();
        bool isEnd();
        int getCodeNum(int no);

    private:
        enum State {
            UNDEFINED,
            INIT,
            PRESSED,
            CHANGE,
            END
        };
        HPolling *mPoller;
        ArmControl * mArmControl;
       // HPuzzle *mPuzzle;

        State mState;
        button_t mPressNo;
        int code;
        int white;
        int color;
        int black;

        int dial;
        int digit;
        int blockCode[5]={0,0,0,0,0};
        int blockCodeInt;

};

#endif
