#include "SpeedSection.h"
#include "util.h"

SpeedSection::SpeedSection(Judge *judge,
                        LineTracer *tracer,
                        StraightWalker *straight,
                        Turn *turn,
                        VirtualTracer *vt,
                        SDFile *sdfile):
    Section(judge, tracer,straight,turn,vt),
    mState(UNDEFINED),
    mSd(sdfile)
{
    mWk[(LINE_>>8)&0xff] =tracer;
    mWk[(STRAIGHT_>>8)&0xff] = straight;
    mWk[(TURN_>>8)&0xff] = turn;
    mWk[(VIRTUAL_>>8)&0xff] = vt;
    mWk[(VIRTUAL_LINE_>>8)&0xff] = vt;
    mWk[(SET_GOAL_PT_>>8)&0xff] = vt;
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


    if((tmp.cmd&0xff00)==SPEED_SEC_CMD::PARAM_END) {
        ((LineTracer*)mSimpleWalker)->resetParam();
        mState = END;
        return;
    }

    int cmd = (tmp.cmd>>8)&0xff;
    /*sprintf(buf,"cmd %d ",cmd);
    msg_f(buf,0);*/

    mActive = mWk[cmd];
    setParam(tmp);

    // セットのみで次のコマンドへ
    int maskedcmd = tmp.cmd&0xff00;
    if(maskedcmd == SET_GOAL_PT_ || maskedcmd==RESET_LENGTH_) {
        param_idx++;
        return;
    }


    ((SectionJudge*)mJudge)->setValue(tmp.cmd,tmp.len);
    mState = TRACE;

    recordCount();

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
    char buf[256];

    int cmd = tmp.cmd & 0xff00;
   /* sprintf(buf,"CMD  %d",cmd);
    msg_f(buf,3);*/
    switch(cmd) {
        case LINE_:

            ((LineTracer*)mActive)->setParam((float)tmp.fwd, (float)tmp.target, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd,  (float)tmp.curve, (float)tmp.ckp);
            ((LineTracer*)mActive)->setBias(0);
            break;
        case VIRTUAL_:
            ((LineTracer*)mSimpleWalker)->resetLinePid();
            ((VirtualTracer*)mActive)->setParam((float)tmp.fwd, (float)0,(float)tmp.target, (float)tmp.kp, (float)tmp.ki, (float)tmp.kd);
            break;
        case VIRTUAL_LINE_:
            ((LineTracer*)mSimpleWalker)->resetLinePid();
            ((VirtualTracer*)mActive)->setParamLine((float)tmp.fwd,  (float)tmp.kp, (float)tmp.ki, (float)tmp.kd);
            break;
        case SET_GOAL_PT_:
            ((VirtualTracer*)mActive)->setGoalPt();
            break;
        case RESET_LENGTH_:
            ((SectionJudge*)mJudge)->resetLength();
            break;
        case TURN_:
            ((LineTracer*)mSimpleWalker)->resetLinePid();
            ((Turn*)mActive)->setFwd(0);
            ((Turn*)mActive)->setTurn(tmp.fwd);
            break;
        case STRAIGHT_:
            ((LineTracer*)mSimpleWalker)->resetLinePid();
             ((StraightWalker*)mActive)->setPWM(tmp.fwd);
            break;
    }
}


