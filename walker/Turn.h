#ifndef _TURN_
#define _TURN_

#include "HPID.h"
#include "Odometry.h"
#include "SimpleWalker.h"
#include "LineTracer.h"
#include "SpeedControl.h"


class Turn : public SimpleWalker    //旋回処理を実施
{
    public:
        Turn(ev3api::Motor& leftWheel,
             ev3api::Motor& rightWheel, 
             Odometry* odo,
            SpeedControl *scon);

        ~Turn();

        void run();

        void setFwd(float f);
        void setTurn(float t);
        void setParam(int direction, float angle);
        void setParam(float angle);
        void setLimit(double limit);
        void setTurnPID(float angle);
        void setLowPWM(float pwm);
        void setStateUndefined();
        void setCurrentAngle(float angle);
        void setTargetAngle(float angle);
        void addTargetAngle(int direction, float angle);
        int isEnd();
        float getTurn();
               
        static const int RIGHT = 1;     //右旋回
        static const int LEFT = -1;     //左旋回

    private:
        enum State {
            UNDEFINED,
            INIT,
            TURNING,
            END
        };
        Odometry* mOdo;             //現在の角度を取得
        HPID* mPID;                 //旋回速度を制御

        State mState;               //状態遷移を管理

        void execUndefined(); 
        void initial();
        void execTurning();
        void end();

        float mTargetAngle;         //回転後の角度
        float mCurrentAngle;        //呼び出し開始時の角度
        float mFwdPow;              //前進値、基本は０でブロックを運ぶときに増加
        float mTurnPow;             //旋回値、基本はPIDを使用して可変でブロックを運ぶときは固定値
        float mLowPWM;              //旋回値の下限


};

#endif
