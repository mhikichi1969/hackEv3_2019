#include "SpeedSection.h"
#include "util.h"

SpeedSection::SpeedSection(Judge *judge,
                        LineTracer *tracer,
                        StraightWalker *straight,
                        Turn *turn,
                        VirtualTracer *vt,
                        SDFile *sdfile):
    Section(judge, tracer,straight,turn),
    mState(UNDEFINED),
    mSd(sdfile)
{
    mWk[LINE_] =tracer;
    mWk[STRAIGHT_] = straight;
    mWk[TURN_] = turn;
    mWk[VIRTUAL_] = vt;
 
}


bool SpeedSection::run()
{
    switch(mState)
    {
        case UNDEFINED:
            execUndefined();
            break;
        case INIT:
            execInit();
            break;
        case TRACE:
            execTrace();
            break;
        case END:
            return true;
    }

    return false;
}

void SpeedSection::execUndefined()
{
    param_idx=0;
   // ev3_speaker_play_tone(NOTE_C4,100);

   // mLineTracer->setParam(70,0,   35*0.65, 20, 22);

   ((LineTracer*)mSimpleWalker)->resetParam();
   mJudge->initOdometry();

    mState = INIT;
}

void SpeedSection::execInit()
{

  //  ev3_speaker_play_tone(NOTE_E4,100);
  char buf[256];
   // sprintf(buf,"ss:%d",course);
    //msg_f(buf,0);

    CParam *mParamPt = mParam[course][param_select];
    CParam tmp = mParamPt[param_idx]; 

    sprintf(buf,"%d %2.0f,%1.1f,%3.1f,%3.1f,%3.1f %2.1f,%2.1f",tmp.cmd,tmp.fwd,tmp.target,tmp.kp,tmp.ki,tmp.kd,tmp.curve,tmp.ckp);
    msg_f(buf,1);

    if(tmp.cmd==SPEED_SEC_CMD::PARAM_END) {
        ((LineTracer*)mSimpleWalker)->resetParam();
        mState = END;
        return;
    }

    mActive = mWk[tmp.cmd];
    setParam(tmp);
   // ((LineTracer*)mSimpleWalker)->resetParam();

   //((LineTracer*)mSimpleWalker)->setParam((float)tmp.fwd, (float)tmp.target, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd,  (float)tmp.curve, (float)tmp.ckp);
   // ((LineTracer*)mRunStyle)->resetParam();

// test
   //((VirtualTracer*)mWk[3])->setParam((float)tmp.fwd, (float)0,(float)25, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd);

    mJudge->setValue(tmp.len);

    recordCount();

    //mSd->open("/ev3rt/apps/Lbase2019.csv");
    mState = TRACE;
}


void SpeedSection::execTrace()
{
    //((LineTracer*)mSimpleWalker)->run();
    
    //((VirtualTracer*)mActive)->run();
    mActive->run();

    if(mJudge->isOK()) {
        //ev3_speaker_play_tone(NOTE_E4,100);
        param_idx++;
        mState = INIT;       
    }
   
}

void SpeedSection::setCourse(int idx)
{
    course =idx;
    bool edge = course==0?true:false;
    ((LineTracer*)mSimpleWalker)->setEdgeMode(edge);
}

void SpeedSection::selectParamNo(int sel)
{
    param_select = sel%max_param;
}

void SpeedSection::setParam(CParam tmp)
{
    switch(tmp.cmd) {
        case LINE_:
            ((LineTracer*)mActive)->setParam((float)tmp.fwd, (float)tmp.target, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd,  (float)tmp.curve, (float)tmp.ckp);
            ((LineTracer*)mActive)->setBias(0);
            break;
        case VIRTUAL_:
            ((VirtualTracer*)mActive)->setParam((float)tmp.fwd, (float)0,(float)tmp.target, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd);
            break;
        case TURN_:
            break;
        case STRAIGHT_:
             ((StraightWalker*)mActive)->setPWM(tmp.fwd);
            break;
    }
}


