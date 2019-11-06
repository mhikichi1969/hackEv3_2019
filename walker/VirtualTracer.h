#ifndef __VIRTUAL_WALKER_H__
#define __VIRTUAL_WALKER_H__

#include "SimpleWalker.h"
#include "HPID.h"
#include "HPolling.h"
#include "Odometry.h"
#include "SpeedControl.h"

class VirtualTracer : public SimpleWalker {
    public:
        VirtualTracer(ev3api::Motor& leftWheel,
                    ev3api::Motor& rightWheel, 
                HPolling* polling,
                Odometry* odo,
                SpeedControl *scon);
        void run();
        void execUndefined();
        void running();
        void turning();

        void execVirtualLineTrace();

        void setParam(double speed, double cx, double cy, double kp, double ki, double kd);
        void setParamLine(double speed,  double kp, double ki, double kd);
        void setParamTurn(double fwd, double turn, double r);
        double calcTurn(double val);
        void setCenter(double x,double y);
        double calcDistance(double current_x,double current_y);
        double calcBias();
        void setGoalPt(double x, double y);
        void setGoalPt();

        void setStartPt(double x, double y);
        void calcLineVector();
        double calcLineDistace(double current_x,double current_y);
        void resetPid();
        void setDirectPwmMode(bool mode);
        void setGyroMode(bool mode);

    private:
        enum State {
            UNDEFINED,
            RUNNING, 
            TURNING,
            LINETRACE,  
            END
        };
        State mState;

        HPolling *mPoller;
        
        HPID *mPID;
        double mTargetSpeed;
        double mBias;
        double center_x;
        double center_y;
        double radius;

        double goal_x;
        double goal_y;

        bool leftTurn;

        double start_x;
        double start_y;

        double line_vec_x;
        double line_vec_y;

        bool mDirectPwmMode;

        bool mTurnMode;
        int mAdjust;

        bool mGyroMode;
};

#endif