#include "VirtualTracer.h"
#include "util.h"
#include "math.h"

#define ODOMETRY
VirtualTracer::VirtualTracer(ev3api::Motor& leftWheel,
                        ev3api::Motor& rightWheel,    
                        HPolling* poller,
                        Odometry* odo,
                        SpeedControl *scon
                    ) 
    : SimpleWalker(leftWheel,rightWheel,odo,scon),
    mPoller(poller),
    mState(UNDEFINED)

{
    mPID = new HPID(0.004);
    mPID->resetParam();
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
    #ifdef ODOMETRY
        double x = mOdo->getX();
        double y = mOdo->getY();
        double angle = mOdo->getAngleDeg();
    #else
        double x = mOdo->getGyroX();
        double y = mOdo->getGyroY();
        double angle = mOdo->getGyroAngle();
   #endif

    x += 7*cos(angle*M_PI/180);
    y += 7*sin(angle*M_PI/180);

    double distance = calcDistance(x,y);


    char buf[256];
    sprintf(buf,"vir:%2.1f,%2.1f,%2.1f",x,y,distance-radius);
    msg_f(buf,2);
    int mTurn = calcTurn(distance-radius);
    setCommandV((int)mTargetSpeed, (int)mTurn);

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

    char buf[256];
    sprintf(buf,"line:%3.1f,%3.1f->%3.1f %d",x,y, distance, mTurn);
    msg_f(buf,12);

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
    #ifdef ODOMETRY
        double x = mOdo->getX();
        double y = mOdo->getY();
        double angle = mOdo->getAngleDeg();
    #else   
        double x = mOdo->getGyroX();
        double y = mOdo->getGyroY();
        double angle = mOdo->getGyroAngle();
    #endif
   // double angle = mOdo->getAngleDeg();
    double front_x = 7*cos(angle*M_PI/180);
    double front_y = 7*sin(angle*M_PI/180);

    double side_x = cy*sin(-angle*M_PI/180);
    double side_y = cy*cos(-angle*M_PI/180);

    leftTurn = cy<0?true:false;

    setCenter(front_x+side_x+x,front_y+side_y+y);
    double target = calcDistance(front_x+x,front_y+y);
    radius = target;

    char buf[256];
    sprintf(buf,"vir:%2.1f:%2.1f,%2.1f,%2.1f,%2.1f",angle,front_x,front_y,side_x,side_y);
    msg_f(buf,3);

    mTargetSpeed = speed;
    mPID->setTarget(0);
    mPID->setKp(kp);
    mPID->setKi(ki);
    mPID->setKd(kd);

    mState = RUNNING;

}

void VirtualTracer::setParamLine(double speed,  double kp, double ki, double kd)
{
    double x = mOdo->getX();
    double y = mOdo->getY();
    setStartPt(x,y);
    calcLineVector();

    char buf[256];
    sprintf(buf,"line:%3.1f,%3.1f->%3.1f,%3.1f",start_x,start_y, goal_x,goal_y);
    msg_f(buf,11);

    mTargetSpeed = speed;
    mPID->setTarget(0);
    mPID->setKp(kp);
    mPID->setKi(ki);
    mPID->setKd(kd);

    mOdo->recordCount();

    mState = LINETRACE;


}

double VirtualTracer::calcTurn(double val)
{
    if(val>2.0) val=2.0;
    if(val<-2.0) val=-2.0;


    double turn =  -mPID->getOperation(val);
    if(leftTurn) turn = -turn;

   // if(mTargetSpeed<0) turn = -turn;

  //  turn += calcBias();

    //if( val<0 ) turn = -20;
    //if(val>=0) turn = 20;

   double t_limit=50;
    t_limit = SimpleWalker::mTargetSpeed>40?SimpleWalker::mTargetSpeed*0.9:SimpleWalker::mTargetSpeed*1.2; //40以上か10～40
    t_limit = SimpleWalker::mTargetSpeed>10?t_limit:SimpleWalker::mTargetSpeed*1.4;  // 10以下
    t_limit = fabs(t_limit);

   if(turn>t_limit) turn = t_limit;
    if(turn<-t_limit) turn = -t_limit;

    return turn;

}

double VirtualTracer::calcBias()
{
    double tread = 14.72;
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
    double len = sqrt((current_x-start_x)*(current_x-start_x)+(current_y-start_y)*(current_y-start_y));
    double vec_x = (current_x-start_x)/len;
    double vec_y = (current_y-start_y)/len;

    char buf[256];
    sprintf(buf,"line:%3.1f,%3.1f->%3.1f,%3.1f",vec_x,vec_y, line_vec_x,line_vec_y);
    msg_f(buf,10);

    double mul = vec_x*line_vec_y-vec_y*line_vec_x;
    int side = mul>0?1:-1;

    double dot = line_vec_x*vec_x+line_vec_y*vec_y;
    double dist = side*len*sqrt(1-dot*dot);

    return dist;
}

