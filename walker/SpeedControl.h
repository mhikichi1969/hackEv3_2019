#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include "HPID.h"
#include "Odometry.h"

class SpeedControl
{
    public:
        SpeedControl(Odometry *odo);
        void setTargetSpeed(double speed);
        int getPwm();
        void resetParam();
        void setMode(bool mode);
        void setBreak(bool brk);
        double getCurrentFwd();
        double getCurrentSpeed();

    private:
        HPID *mPid;
        Odometry *mOdo;

        double mTargetSpeed;
        int mForward; //現在PWM値
        double mCurrentSpeed;

        bool mBreake_flag;
        bool mMode_flag;//setComandVだとtrue、setComandだとfalse


};
#endif
