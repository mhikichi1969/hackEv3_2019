#ifndef _JUDGE_H_
#define  _JUDGE_H_

#include "HPolling.h"
#include "Odometry.h"
//#include "Color.h"
#include "BingoEnum.h"
#include "Flag.h"

class Judge {
    public:
        Judge(HPolling *polling,
            Odometry *odometry);
        //virtual bool isOK()=0;
        void setHSV();
        void setTargetHSV(float h, float s, float v);
        void setColor(double col);
        void setPermitAngle(float angle);
        void recordCount();
        void resetLength();
        void initOdometry();

        virtual void setAngleParam(Flag::End endFlag);
        void setParam(double fwd, double target, double len, 
                      double turn, Flag::Method runFlag, Flag::End endFlag);

        virtual bool angleCheck();

        bool isOK();
        void resetParam();
        void addTargetParam(double t);

        //COLOR searchColor(double h, double s);
        hsv_t getHSV();

        static constexpr double VALNULL = 9999.99d;     

    protected:
        HPolling *mPoller;
        Odometry *mOdo;
        hsv_t mHSV;
        hsv_t mTargetHSV;

        //double mCol;
        double mVal;
        //double mVal1;
        //double mVal2;

        double mHSVRange;
        float mPermitAngle;
        bool mAngFlag;          //右旋回の時にtrue、左旋回の時にfalse
        bool mBackFlag;          //後退の時にtrue、前進の時にfalse
        float mBlkRate;         //黒と判断する基準値、-1が黒、1が白
        double mStartAngle;

        double mFwd;
        double mTarget;
        double mLen;
        double mTurn;
        int mCnt;
        Flag::Method mRunFlag;
        Flag::End mEndFlag;

};

#endif