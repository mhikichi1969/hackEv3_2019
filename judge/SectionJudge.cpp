#include "SectionJudge.h"

SectionJudge::SectionJudge(HPolling *polling,
                Odometry *odometry) 
            : Judge( polling,odometry)
{                
}
/*
bool SectionJudge::isOK()
{
    bool f = false;

    if(mVal!=Judge::VALNULL){
        //ライントレースした距離を見る場合
        //直進した距離を見る場合
        if(mOdo->getLength()>mVal){
            f = true;
            msg_f("Judge:Length",7);
        }
        //else 
        //    f = false;
    }else if(mVal1!=Judge::VALNULL && 
             mVal2!=Judge::VALNULL){
        //角度を見る場合
        //msg_f("Judge:angle",3);

        //if(fabs(mOdo->getAngle()-mVal1) < mPermitAngle*M_PI/180.0f &&
        if(fabs(mOdo->getGyroAngle()-mVal1) < mPermitAngle ){//&&
           //fabs(mVal2) < 10.0d){
            msg_f("SJudge:angle:TRUE",7);
            f = true;
        }else{
            msg_f("SJudge:angle:ELSE",7);

        }

        //if(mOdo->getAngle()<mVal1 && mOdo->getAngle()>mVal2)
        //    f = true;
        //else 
        //    f = false;
    }else if(mVal==Judge::VALNULL && 
             mVal1==Judge::VALNULL && 
             mVal2==Judge::VALNULL){
        msg_f("Judge:GET_COLOR",7);
        
        getHSV();
        //msg_f("Judge:GET_COLOR_10",7);

        //if(mSetHsv.h*(1+mHSVRangePoint) > mGetHsv.h &&
        //   mSetHsv.h*(1-mHSVRangePoint) < mGetHsv.h &&
        if(mSetHsv.h + mHSVRange > mGetHsv.h &&
           mSetHsv.h - mHSVRange < mGetHsv.h &&
           mSetHsv.s < mGetHsv.s &&
           mSetHsv.v < mGetHsv.v ){

            msg_f("Judge:GET_COLOR:END1",7);
        	ev3_speaker_play_tone(NOTE_F4,20);
            f = true;
        //msg_f("Judge:GET_COLOR_1",8);
        
        //}else if(mSetHsv.h*(1+mHSVRangePoint) > 360 &&
        //         mSetHsv.h*(1+mHSVRangePoint)-360 > mGetHsv.h &&
        }else if(mSetHsv.h + mHSVRange > 360 &&
                 mSetHsv.h + mHSVRange - 360 > mGetHsv.h &&
                 0 < mGetHsv.h &&
                 mSetHsv.s < mGetHsv.s &&
                 mSetHsv.v < mGetHsv.v ){

            msg_f("Judge:GET_COLOR:END2",7);
        	ev3_speaker_play_tone(NOTE_F4,20);
            f = true;
        //msg_f("Judge:GET_COLOR_2",8);
        }
        //else 
        //    f = false;
    }
    
    return f;
}*/

