#include "HPID.h"
#include "util.h"

HPID::HPID() 
{
    limit = 100;
    diff[0]=diff[1]=0.0;
    integral=0;
    DELTAT=0;
    
}
HPID::HPID(double delta) {
    limit = 100;
    diff[0]=diff[1]=0.0;
    integral=0;
    DELTAT=delta;

    for(int i=0;i<old_max;i++) 
        old[i]=0;
}
HPID::~HPID() 
{
}

void HPID::setLimit(double limit) 
{
    this->limit = limit;
}
void HPID::setTarget(double t)
{
    this->target = t;
   // diff[0]=diff[1]=t;
}

double HPID::getOperation(double value)
{

   char buf[256];
    if (debug) {
        sprintf(buf,"pid:(%3.1f-%3.1f), i:%3.1f",target,value,integral);
        msg_f(buf,11);
    }

   
    diff[0]=diff[1];
    diff[1] = target-value;
    double prev_i=integral;
    if(DELTAT==0) {
        delta = (diff[1]-diff[0])/0.004;
        //integral -= old[old_cnt];
        //old[old_cnt] = (diff[0]+diff[1])/2.0f*0.004;
        old[old_cnt] = (diff[0]+diff[1])/2.0*0.004;
        integral+=old[old_cnt++];
    } else {
        delta = (diff[1]-diff[0])/DELTAT;
        //integral -= old[old_cnt];
        //old[old_cnt] = (diff[0]+diff[1])/2.0f*0.004;
        old[old_cnt] = (diff[0]+diff[1])/2.0*DELTAT;
        integral+=old[old_cnt++];      
    }

    // 積分値のオーバーを防ぐ
    if (integral>10.0) 
        integral=10.0;
    if (integral<-10.0) 
        integral=-10.0;


    if(old_cnt>=old_max)
        old_cnt=0;

   // if( integral>50 ) integral= 50;
   // if( integral<-50 ) integral= -50;
 
    double val = diff[1]*Kp + delta*Kd + integral*Ki;
    if(val>limit) 
        val=limit;
    if(val<-limit)
        val=-limit;
    
    return val;
}

void HPID::setKp(double kp)
{
    Kp = kp;
}

void HPID::setKi(double ki)
{
    Ki = ki;
}

void HPID::setKd(double kd)
{
    Kd = kd;
}

double HPID::getDiff() 
{
    return diff[1];
}

double HPID::getIntegral()
{
    return integral;
}

void HPID::resetParam()
{
    diff[0]=diff[1]=0.0f;
    integral=0;
    //Ki=Kd=0;
}

double HPID::getTarget()
{
    return target;
}

void HPID::setDeltaT(double delta)
{
    DELTAT = delta;
}