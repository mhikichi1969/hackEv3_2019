#include "DeviceError.h"
#include "util.h"

DeviceError::DeviceError(HPolling *poller):
    mPoller(poller),
    mState(UNDEFINED)
{   
    angle_cnt=0;

}

bool DeviceError::run()
{
    switch(mState) {
        case UNDEFINED:
            execUndefined();
        break;
        case INIT:
        break;
        case CHECK:
            execCheck();
        break;
        case STOP:
        break;

    }
    return false;
}
void DeviceError::execUndefined()
{
    mState = CHECK;
}
void DeviceError::execCheck()
{
    char buf[256];
    sprintf(buf,"battery %d",mPoller->getVoltage());
    msg_f(buf,12);

    checkGyro();
    if(angle_cnt>5) {
      //  ev3_speaker_play_tone(NOTE_C4,500);
        angle_cnt=0;
    }

}


void DeviceError::checkGyro()
{
    static int prev_dir=0;

    int angle = (int)mPoller->getGyroAngle();
    
    int dir = (angle>prev_angle)?1:((angle<prev_angle)?-1:0);
    prev_angle= angle;

    
    if(prev_dir*dir<=0)
        angle_cnt=0;
    else
    {
        angle_cnt++;
    }

    prev_dir = dir;
    
}

void DeviceError::stopCheck()
{
    mState = STOP;
}
