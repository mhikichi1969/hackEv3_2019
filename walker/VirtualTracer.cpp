#include "VirtualTracer.h"
#include "util.h"
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
    }
}


void VirtualTracer::execUndefined()
{
    msg_f("virtual",0);
    mState = RUNNING;
}

void VirtualTracer::running()
{
    double x = mOdo->getGyroX();
    double y = mOdo->getGyroY();

    double angle = mPoller->getGyroAngle();
    x += 3*cos(angle*M_PI/180);
    y += 3*sin(angle*M_PI/180);

    double distance = calcDistance(x,y);


    //char buf[256];
    //sprintf(buf,"vir:%2.1f,%2.1f,%2.1f",x,y,distance-radius);
   // msg_f(buf,2);
    int mTurn = calcTurn((distance-radius)/2.0);
    setCommandV((int)mTargetSpeed, (int)mTurn);

    SimpleWalker::run();

}



void VirtualTracer::setParam(double speed, double cx,double cy, double kp, double ki, double kd)
{
    mOdo->resetAngle();
    mOdo->resetLength();
    double x = mOdo->getGyroX();
    double y = mOdo->getGyroY();

    double angle = mPoller->getGyroAngle();
    x += 7*cos(-angle*M_PI/180);
    y += 7*sin(-angle*M_PI/180);

    leftTurn = cy<0?true:false;

    setCenter(cx,cy);
    double target = calcDistance(x,y);
    radius = target;

    char buf[256];
    sprintf(buf,"vir:%2.1f,%2.1f,%2.1f",cx,cy,radius);
    msg_f(buf,3);

    mTargetSpeed = speed;
    mPID->setTarget(0);
    mPID->setKp(kp);
    mPID->setKi(ki);
    mPID->setKd(kd);

}

double VirtualTracer::calcTurn(double val)
{
    if(val>1.0) val=1.0;
    if(val<-1.0) val=-1.0;


    double turn =  -mPID->getOperation(val);
    if(leftTurn) turn = -turn;

    if(mTargetSpeed<0) turn = -turn;

    turn += calcBias();

    //if( val<0 ) turn = -20;
    //if(val>=0) turn = 20;
    if(turn>100.0) turn = 100.0;
    if(turn<-100.0) turn = -100.0;
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
