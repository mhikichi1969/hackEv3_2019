#include "SectionJudge.h"

#include "Flag.h"

SectionJudge::SectionJudge(HPolling *polling,
                Odometry *odometry) 
            : Judge( polling,odometry)
{                
}

void SectionJudge::setValue(int cmd_judge,double val)
{   
    int cmd = cmd_judge&0xff00;
    mEndFlag = (Flag::End)(cmd_judge&0xff);

    mTarget = val;      

    switch(mEndFlag) {
        case Flag::END_LEN:
            mBackFlag = val<mLen; 
            mVal = val;
            mLen = val;
            break;
         case Flag::END_ANG:
         case Flag::END_ANG2:
            setAngleParam(mEndFlag);
            break;
    }
}


void SectionJudge::setAngleParam(Flag::End endFlag)
{
       /* char buf[256];
         sprintf(buf,"SJ:setAP %3.1f,%3.1f",mOdo->getAngleDeg(),mTarget);
        msg_f(buf,8);*/

        switch(endFlag) {

            case Flag::END_ANG:
                mStartAngle = mOdo->getAngleDeg();
            case Flag::END_ANG2:
                mTarget += mStartAngle;
                if(mTarget < mOdo->getAngleDeg()){
                    mAngFlag = false;
                }else{
                    mAngFlag = true;
                }
        }
}


bool SectionJudge::angleCheck()
{
   /* char buf[256];
    sprintf(buf,"angleCheck %3.1f:%3.1f",mOdo->getAngleDeg(),mTarget);
    msg_f(buf,6);*/
    bool f=false;
    if(mAngFlag && mOdo->getAngleDeg() >= mTarget - mPermitAngle ){
        //msg_f("SectionJudge:angle:TRUE1",7);
        //msg_f("Judge:angle:TRUE1",2);
        f = true;
    }else if(!mAngFlag && mOdo->getAngleDeg() <= mTarget + mPermitAngle ){
        //msg_f("SectionJudge:angle:TRUE2",7);
        //msg_f("Judge:angle:TRUE2",2);
        f = true;
    }else{
        //msg_f("Judge:angle:ELSE",7);
    }

    return f;

}

void SectionJudge::resetLength()
{
    mLen=0;
    mOdo->resetLength();
}