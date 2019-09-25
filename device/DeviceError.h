#ifndef __DEVICE_ERROR_H__
#define __DEVICE_ERROR_H__
#include "HPolling.h"

class DeviceError
{
    public:
        DeviceError(HPolling *poller);
        bool run();
        void execUndefined();
        void execCheck();
        void checkGyro();
        void stopCheck();


    private:
        enum State{
            UNDEFINED,
            INIT,
            CHECK,
            STOP

        };
        State mState;

        HPolling *mPoller;
        
        int prev_angle;
        int angle_cnt;


};

#endif
