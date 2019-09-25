#ifndef _STRAIGHTWALKER_
#define _STRAIGHTWALKER_

#include "HPID.h"
#include "Odometry.h"
#include "SimpleWalker.h"
#include "LineTracer.h"
#include "SpeedControl.h"

class StraightWalker : public SimpleWalker    //直進処理を実施
{
    public:
        StraightWalker(ev3api::Motor& leftWheel,
             ev3api::Motor& rightWheel, 
             Odometry* odo,
            SpeedControl *scon);

        ~StraightWalker();

        void run();

        void setParam(float length);
        void setLimit(double limit);
        void setPID();
        void setPWM(int pwm);
        void setCurrentLength(float length);
        void setTargetLength(float length);
        void setStateUndefined();
        void setFBFlag(bool f);
        int isEnd();
       
    private:
        enum State {
            UNDEFINED,
            INIT,
            WALKING,
            END
        };
       // Odometry* mOdo;             //距離を取得
        HPID* mPID;                 //左右モーターの回転数の差を制御

        State mState;               //状態遷移を管理

        void execUndefined(); 
        void initial();
        void execWalking();
        void end();

        float mDiffRecord;          //左モーターと右モーターの差分
        float mCurrentLength;       //起動時にスタートから走っている距離
        float mTargetLength;        //走行したい距離
        int mPWM;                   //直進時のPWM値

        bool mBackFlag;
};

#endif
