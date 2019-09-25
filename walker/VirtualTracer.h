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

        void setParam(double speed, double cx, double cy, double kp, double ki, double kd);
        double calcTurn(double val);
        void setCenter(double x,double y);
        double calcDistance(double current_x,double current_y);
        double calcBias();

    private:
        enum State {
            UNDEFINED,
            RUNNING,   
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

};

#endif