#include "VirtualTracer.h"
#include "util.h"
#include "math.h"

#define M_PI 3.14159265358979323846

#define ODOMETRY
VirtualTracer::VirtualTracer(ev3api::Motor& leftWheel,
                        ev3api::Motor& rightWheel,    
                        HPolling* poller,
                        Odometry* odo,
                        SpeedControl *scon
                    ) 
    : SimpleWalker(leftWheel,rightWheel,odo,scon),
    mPoller(poller),
    mDirectPwmMode(false),
    mState(UNDEFINED)

{
    mPID = new HPID();
    mPID->resetParam();

    mAdjust=0;
    mTurn=0;

    mGyroMode = false;
}

void VirtualTracer::run()
{
    switch(mState)
    {
        case UNDEFINED:
            execUndefined();
            break;
        case RUNNING:
            running();
            break;
        case TURNING:
            turning();
            break;
        case LINETRACE:
            execVirtualLineTrace();
            break;
    }
}


void VirtualTracer::execUndefined()
{
    mState = RUNNING;
}

void VirtualTracer::running()
{
    double x,y,angle;
    if(mGyroMode) {
        x = mOdo->getGyroX();
        y = mOdo->getGyroY();
        angle = mOdo->getGyroAngle();

    } else {
        x = mOdo->getX();
        y = mOdo->getY();
        angle = mOdo->getAngleDeg();
    }
    double noselen = mDirectPwmMode?4.0:7.0; 


    int dir = mTargetSpeed>=0?1:-1;
    x += dir*noselen*cos(angle*M_PI/180);
    y += dir*noselen*sin(angle*M_PI/180);

    double distance = calcDistance(x,y);

    static int cnt=0;
    static char buf[256];

    int mTurn = calcTurn(distance-radius);
    int offset=mTurnOffset;
    if(mDirectPwmMode)            
        mBias = leftTurn?-(mTargetSpeed-offset):(mTargetSpeed-offset);
    else
        mBias = offset;

 //   sprintf(buf,"vir:%2.1f,%2.1f, r: %2.1f diff:%2.3f spd:%2.1f bias %2.1f,turn:%d mode:%d",x,y,radius, distance-radius,mTargetSpeed,mBias,mTurn,mDirectPwmMode);
 //  msg_f(buf,2);

//    syslog(LOG_NOTICE,"turn offs %d",mTurnOffset);


    if(mDirectPwmMode) {           
        if(mTurn>25) {
           // ev3_speaker_play_tone(NOTE_F5,50);
            mTurn=25;
        }
        if(mTurn<-25) {
           // ev3_speaker_play_tone(NOTE_C4,50);
            mTurn=-25;
        }
          //  sprintf(buf,"vir:set:%d %d",(int)mTargetSpeed, (int)mBias+mTurn);
        setCommand((int)mTargetSpeed, (int)mBias+mTurn);
    } else {
        setCommandV((int)mTargetSpeed, (int)mBias+mTurn);
    }

    SimpleWalker::run();

}

void VirtualTracer::turning()
{
    double x,y,angle;
    if(mGyroMode) {
        x = mOdo->getGyroX();
        y = mOdo->getGyroY();
        angle = mOdo->getGyroAngle();
    } else {
        x = mOdo->getX();
        y = mOdo->getY();
        angle = mOdo->getAngleDeg();
    }

    double distance = calcDistance(x,y);
    int mTurn = calcTurn(distance);

    if(distance>0.2) 
        mAdjust++;
    if(distance<-0.2)
        mAdjust--; 
    if(mAdjust>4) mAdjust=4;
    if(mAdjust<-4) mAdjust=-4;

    //setCommandV((int)mTargetSpeed, (int)mBias+mAdjust);
    setCommandV((int)mTargetSpeed, (int)mTurn);

  /*  static char buf[256];
    sprintf(buf,"vir:%2.1f,%2.1f, r: %2.1f turn %d  bias %2.1f,adj:%d",x,y,distance,mTurn,mBias,mAdjust);
   msg_f(buf,2);
   */

    SimpleWalker::run();



}
void VirtualTracer::execVirtualLineTrace()
{
    double x = mOdo->getX();
    double y = mOdo->getY();
    double angle = mOdo->getAngleDeg();

    int dir = (mTargetSpeed>0)?1:-1;
    x += dir*7*cos(angle*M_PI/180);
    y += dir*7*sin(angle*M_PI/180);

    double distance = calcLineDistace(x,y);


    int mTurn = calcTurn(distance);
    mBias = mTargetSpeed;

   /* static char buf[256];
    sprintf(buf,"execVirtualLineTrace:%3.1f,%3.1f->%3.1f %d",x,y, distance, mTurn);
    msg_f(buf,12);
    */
    

    if(mDirectPwmMode)
        setCommand((int)mTargetSpeed, (int)mTurn+mBias);
    else
        setCommandV((int)mTargetSpeed, (int)mTurn);

    SimpleWalker::run();
}


/*  仮想トレースの円の中心を指定する
cx 前方距離、cy +:右距離 -:左距離 
*/
void VirtualTracer::setParam(double speed, double cx,double cy, double kp, double ki, double kd)
{
    // xは進行方向
   // mOdo->resetAngle();
   // mOdo->resetLength();
    double x,y,angle;
    if(mGyroMode) {
        x = mOdo->getGyroX();
        y = mOdo->getGyroY();
        angle = mOdo->getGyroAngle();
    } else {
        x = mOdo->getX();
        y = mOdo->getY();
        angle = mOdo->getAngleDeg();
    }
   // double angle = mOdo->getAngleDeg();
    int dir = speed>=0?1:-1;
    double noselen = mDirectPwmMode?3.0:7.0;   // 4.0

    double front_x = dir*noselen*cos(angle*M_PI/180);
    double front_y = dir*noselen*sin(angle*M_PI/180);

    double side_x = cy*sin(-angle*M_PI/180);
    double side_y = cy*cos(-angle*M_PI/180);

    leftTurn = (dir*cy<0)?true:false;

    setCenter(front_x+side_x+x,front_y+side_y+y);
    double target = calcDistance(front_x+x,front_y+y);
    radius = target;

  /*  char buf[256];
    sprintf(buf,"vir:%2.1f:%2.1f,%2.1f,%2.1f,%2.1f",angle,front_x,front_y,side_x,side_y);
    msg_f(buf,3);
    */

    mTargetSpeed = speed;
    mPID->setTarget(0);
    mPID->setKp(kp);
    mPID->setKi(ki);
    mPID->setKd(kd);

    mState = RUNNING;

}

void VirtualTracer::setParamLine(double speed,  double kp, double ki, double kd, double angle)
{
    double x = mOdo->getX();
    double y = mOdo->getY();

    setAngleAndPos(angle,x,y);
    leftTurn=true; // 直線制御には旋回用のデフォルト値を使う

    static char buf[256];
    sprintf(buf,"line:%3.1f,%3.1f->%3.1f,%3.1f",start_x,start_y, target_x,target_y);
    msg_f(buf,11);

    mTargetSpeed = speed;
    mPID->setTarget(0);
    mPID->setKp(kp);
    mPID->setKi(ki);
    mPID->setKd(kd);
    mPID->resetParam();

    mOdo->recordCount();

    mState = LINETRACE;


}

void VirtualTracer::setParamTurn(double fwd, double turn, double r)
{
    double x,y,angle;
    if(mGyroMode) {
        x = mOdo->getGyroX();
        y = mOdo->getGyroY();
        angle = mOdo->getGyroAngle();
    } else {
        x = mOdo->getX();
        y = mOdo->getY();
        angle = mOdo->getAngleDeg();
    }

   // double angle = mOdo->getAngleDeg();
    int dir = fwd>=0?1:-1;
    double side_x = r*sin(-angle*M_PI/180);
    double side_y = r*cos(-angle*M_PI/180);

    leftTurn = (dir*r<0)?true:false;

    setCenter(side_x+x,side_y+y);
    double target = calcDistance(x,y);
    radius = target;

    mTargetSpeed = fwd;
    mBias=turn;
    mAdjust=0;

    mState = TURNING;

}

double VirtualTracer::calcTurn(double val)
{
    static char buf[256];
    if(val>2.0) val=2.0;
    if(val<-2.0) val=-2.0;


    double turn =  -mPID->getOperation(val);
   // sprintf(buf,"calcTurn:%f, %f",val,turn);
   // msg_f(buf,1);
    if(leftTurn) turn = -turn;

   // if(mTargetSpeed<0) turn = -turn;
  //  turn += calcBias();
    //if( val<0 ) turn = -20;
    //if(val>=0) turn = 20;

   double t_limit=50;
   // t_limit = fabs(SimpleWalker::mTargetSpeed)>40?fabs(SimpleWalker::mTargetSpeed)*0.9:fabs(SimpleWalker::mTargetSpeed)*1.8; //40以上か10～40
   // t_limit = fabs(SimpleWalker::mTargetSpeed)>8?t_limit:fabs(SimpleWalker::mTargetSpeed)*5.0;  // 8以下
    t_limit = fabs(t_limit);

   if(turn>t_limit) turn = t_limit;
    if(turn<-t_limit) turn = -t_limit;

    return turn;

}

double VirtualTracer::calcBias()
{
    double tread = 19.01; //14.72
    double a = radius-tread/2.0;
    double b = radius+tread/2.0;
    double fwd_rate = (a+b)/2.0;
    double turn_rate = fwd_rate-a;

    double bias = turn_rate*mForward/fwd_rate;
    if(leftTurn)
        bias = -bias;
    return bias;
}

void VirtualTracer::setCenter(double x,double y)
{
    center_x = x;
    center_y = y;

}


double VirtualTracer::calcDistance(double current_x,double current_y)
{
    return sqrt((current_x-center_x)*(current_x-center_x)+(current_y-center_y)*(current_y-center_y));
}

void VirtualTracer::setGoalPt(double x, double y)
{
    goal_x =  x;
    goal_y = y;
}

void VirtualTracer::setGoalPt()
{
    setGoalPt(mOdo->getX(),mOdo->getY());
    //setGoalPt(100,40);
}


void VirtualTracer::setStartPt(double x, double y)
{
    start_x =  x;
    start_y = y;
}

void VirtualTracer::calcLineVector()
{
    double len = sqrt((goal_x-start_x)*(goal_x-start_x)+(goal_y-start_y)*(goal_y-start_y));
    line_vec_x = (goal_x-start_x)/len;
    line_vec_y = (goal_y-start_y)/len;
}

double VirtualTracer::calcLineDistace(double current_x,double current_y)
{
    // 進行方向右は- 、左が+ かな？つまり右エッジ相当
    double dist = (target_y-start_y)*current_x - (target_x-start_x)*current_y + target_x*start_y- target_y*start_x;

    
    /*static  char buf[256];
    sprintf(buf,"calcLineDistace:c_:%3.1f,%3.1f s_:%3.1f,%3.1f->t_:%3.1f,%3.1f : %3.1f",current_x,current_y, start_x, start_y, target_x, target_y,dist);
    msg_f(buf,10);
    */

    return -dist;
}

void VirtualTracer::resetPid()
{
    mPID->resetParam();
}

void VirtualTracer::setDirectPwmMode(bool mode)
{
    mDirectPwmMode = mode;
}

void VirtualTracer::setGyroMode(bool mode)
{
    mGyroMode = mode;
}

void VirtualTracer::setTurnOffset(double offset)
{
    mTurnOffset = offset;
}

void VirtualTracer::setAngleAndPos(double a, double x, double y)
{
    // angleはx軸方向が0, 時計回りが＋
    double current_anlge = a;

    target_x = x+cos(a*M_PI/180);
    target_y = y+sin(a*M_PI/180);
    start_x = x;
    start_y = y;


}
