#include "Judge.h"
#include "Const.h"

Judge::Judge(HPolling *polling,
            Odometry *odometry):
            mPoller(polling),
            mOdo(odometry)
{
    mVal = VALNULL;         //距離
    //mVal1 = VALNULL;        //角度許容上限//目標角度
    //mVal2 = VALNULL;        //角度許容下限//旋回値
    mHSVRange = 50.0d;
    mPermitAngle = 0.0f;
    mAngFlag = false;
    mBackFlag = false;
    mBlkRate = -0.8;    
    mStartAngle = 0.0d;
    mLen = 0;

   resetParam();
}

void Judge::setHSV()
{   
    mPoller->getHSV(mHSV);
}

hsv_t Judge::getHSV()
{   
    return mHSV;
}

void Judge::setTargetHSV(float h, float s, float v)
{   
    mTargetHSV.h = h;
    mTargetHSV.s = s;
    mTargetHSV.v = v;
}

void Judge::setColor(double col)
{   
    switch ((COLOR)col)
    {
    case COLOR::RED :
        //setHSV(358.0 , 0.9 , 14.0);//試走会１
        setTargetHSV(H_RED_C , S_RED_C ,0);
        break;
    case COLOR::GREEN :
        //setHSV(125.0 , 0.5 , 9.0);//試走会１
        setTargetHSV(H_GREEN_C , S_GREEN_C , 0.0);//DS蛍光灯あり
        //setHSV(110.0 , 0.3 , 7.0);//DS蛍光灯無し
        break;
    case COLOR::BLUE :
        setTargetHSV(H_BLUE_C , S_BLUE_C , 0.0);
        break;
    case COLOR::YELLOW :
        //setHSV(38.0 , 0.5 , 10.0);//試走会１
        //setHSV(35.0 , 0.5 , 10.0);//DS蛍光灯あり
        setTargetHSV(H_YELLOW_C , S_YELLOW_C , 0.0);//DS蛍光灯あり
        break;
    
    case COLOR::NONE :
    default:
        break;
    }
}


/* 許容誤差角度設定
 * 誤差を許容する角度を指定
 * 
 * angle            :float      :許容する誤差（°）
*/
void Judge::setPermitAngle(float angle)
{   
    mPermitAngle = angle;
}

/* レコードカウント実施
 * 走行体の向きの記憶に使用
*/
void Judge::recordCount()
{
    mOdo->recordCount();
}

/* 走行距離リセット
 * 距離の終了条件の設定前に実行する必要がある
*/
void Judge::resetLength()
{
    mOdo->resetLength();
}

void Judge::initOdometry()
{
    mPoller->resetGyroAngle();
    mOdo->reset();
}

int Judge::getTurnDirection(double target )
{
    return target+mStartAngle>mTmpStartAngle?1:-1;
}

void Judge::setAngleParam(Flag::End endFlag)
{
        #if GYRO
            double angle = mOdo->getGyroAngle();
        #else
            double angle = mOdo->getAngleDeg();
        #endif

        switch(endFlag) {
            case Flag::END_ANG:
                mStartAngle = angle;
            case Flag::END_ANG2:
                mTmpStartAngle = angle;
                mTarget += mStartAngle;
                if(mTarget < angle){
                    mAngFlag = false;
                }else{
                    mAngFlag = true;
                }
        }
}

void Judge::setParam(double fwd, double target, double len, 
                     double turn, Flag::Method runFlag, Flag::End endFlag)
{
        mFwd = fwd;
        mTarget = target;       //旋回角度、終了色
        mLen = len;             //進行距離
        mTurn = turn;
        mRunFlag = runFlag;
        mEndFlag = endFlag;     //終了条件

        char buf[256];

        sprintf(buf,"Judge:setP %2.1f,%2.1f,%2.1f, %d,%d,",fwd,target,len,runFlag,endFlag);
        msg_f(buf,5);

        switch (mEndFlag)
        {
        case Flag::END_LEN:
            //sprintf(buf,"Judge:LEN:mLen: %3.1f",mLen);
            //msg_f(buf,5);
            if(mLen < 0){
                mBackFlag = true;
                //sprintf(buf,"Judge:LEN:mLen:True: %3.1f",mLen);
                //msg_f(buf,5);
            }else{
                mBackFlag = false;
                //sprintf(buf,"Judge:LEN:mLen:False: %3.1f",mLen);
                //msg_f(buf,5);
            }
            break;
        case Flag::END_COL:
            setColor(mTarget);
            break;
        
        case Flag::END_ANG:
        case Flag::END_ANG2:
            setAngleParam(mEndFlag);
            break;
        
        case Flag::END_ARM:
            mTarget += mPoller->getArmCount();
            break;
        
        case Flag::END_CNT:
            mCnt = 0;
            break;

        case Flag::END_BLK:
        default:
            break;
        }
}

bool Judge::angleCheck()
{
    char buf[256];
  //  msg_f("Judge:angle:",2);
#if GYRO
    double angle = mOdo->getGyroAngle();
#else
    double angle = mOdo->getAngleDeg();
#endif
 //   sprintf(buf,"angleCheck:%f",angle);
//    msg_f(buf,2);

    bool f=false;
    if(mAngFlag && angle >= mTarget - mPermitAngle ){
        //msg_f("Judge:angle:TRUE1",7);
        //msg_f("Judge:angle:TRUE1",2);
        ev3_speaker_play_tone(NOTE_B5,50);

        f = true;
    }else if(!mAngFlag && angle<= mTarget + mPermitAngle ){
        //msg_f("Judge:angle:TRUE2",7);
        //msg_f("Judge:angle:TRUE2",2);
        ev3_speaker_play_tone(NOTE_B5,50);

        f = true;
    }else{
        //msg_f("Judge:angle:ELSE",7);
    }

    return f;
}

bool Judge::isOK()
{
    bool f = false;

    switch (mEndFlag){
        case Flag::END_LEN:
            //msg_f("Judge:CHECK_Length",7);
            
          /*  char buf[256];
            sprintf(buf,"Judge:LEN:G: %3.1f , L: %3.1f",mOdo->getLength() , mLen);
            msg_f(buf,2);
            if(mBackFlag){
                msg_f("Judge:mBackFlag:TRUE",3);
            }
            */
            if(!mBackFlag && mOdo->getLength() > mLen){
                //前進処理
                f = true;
                //msg_f("Judge:Length",7);
            }else if(mBackFlag && mOdo->getLength() < mLen){
                //後退処理
                f = true;
                //msg_f("Judge:Length2",7);
            }else{
                //msg_f("Judge:Length:ELSE",4);    
            }
            break;

        case Flag::END_ANG:
        case Flag::END_ANG2:
            f = angleCheck();
            break;

        case Flag::END_ARM:
            //msg_f("Judge:CHECK_ARM",7);

            if(mTarget < mPoller->getArmCount()){
                //msg_f("Judge:END_ARM",7);
                f = true;
            }
            break;

        case Flag::END_COL:
            //msg_f("Judge:CHECK_COLOR",7);
        
            setHSV();

            //char buf[256];
            //sprintf(buf,"H: %2.1f , S: %2.1f , V: %2.1f",mHSV.h,mHSV.s,mHSV.v);
            //msg_f(buf,8);
            //sprintf(buf,"H: %2.1f , S: %2.1f , V: %2.1f , R: %2.1f",mTargetHSV.h,mTargetHSV.s,mTargetHSV.v,mHSVRange);
            //msg_f(buf,7);
            /*
            if(mTargetHSV.h + mHSVRange > mHSV.h &&
               mTargetHSV.h - mHSVRange < mHSV.h &&
               mTargetHSV.s < mHSV.s &&
               mTargetHSV.v < mHSV.v ){

                //msg_f("Judge:GET_COLOR:END1",7);
            	//ev3_speaker_play_tone(NOTE_F4,20);
                f = true;
        
            }else if(mTargetHSV.h + mHSVRange > 360 &&
                     mTargetHSV.h + mHSVRange - 360 > mHSV.h &&
                     0 < mHSV.h &&
                     mTargetHSV.s < mHSV.s &&
                     mTargetHSV.v < mHSV.v ){

                //msg_f("Judge:GET_COLOR:END2",7);
            	//ev3_speaker_play_tone(NOTE_F4,20);
                f = true;
            }
            */
           if (getHueDistance(mTargetHSV.h,mHSV.h)<mHSVRange &&  
                    mTargetHSV.s < mHSV.s &&
                     mTargetHSV.v < mHSV.v) {
                         f=true;
             }

            break;

        case Flag::END_BLK:
            //msg_f("Judge:CHECK_BLACK",7);
            if(mPoller->getBrightnessRate() <= mBlkRate){
                f = true;
            }
            break;

        case Flag::END_CNT:
            //msg_f("Judge:END_CNT",7);
            if(mCnt>mTarget){
                f = true;
            }
            mCnt++;
            break;
        
        case Flag::END_ALL:
            //msg_f("Judge:END_ALL",7);
            f = true;
            break;
        
        case Flag::END_UDF:
            //msg_f("Judge:END_UDF",7);
            break;
        default:
            //msg_f("Judge:END",7);
            break;
    }

  /*  if(f)
        msg_f("Judge:OK:",2);
*/
    return f;
}

void Judge::resetParam()
{   
    mFwd = 0;
    mTarget = 0;
    mLen = 0;
    mTurn = 0;
    mRunFlag = Flag::RUN_UNDEF;
    mEndFlag = Flag::END_UDF;
}

void Judge::addTargetParam(double t)
{   
    mTarget += t;
}

/*
COLOR Judge::searchColor(double h, double s)
{   
    COLOR c = COLOR::BLACK;
    if(s>0.2){
        if(h>300){
            c = COLOR::RED;
        }else if(h>200){
            c = COLOR::BLUE;
        }else if(h>100){
            c = COLOR::GREEN;
        }else if(h>15){
            c = COLOR::YELLOW;
        }else{
            c = COLOR::RED;
        }
    }
    return c;
}*/

double Judge::getHueDistance(double ang1,double ang2)
{
    double diff = fabs(ang1-ang2);
    diff = diff>180?360-diff:diff;

    return diff;
}
