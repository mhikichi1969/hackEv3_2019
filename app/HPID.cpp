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

   static char buf[256];

    double time = 0.01;
    diff[0]=diff[1];
    diff[1] = target-value;
    double prev_i=integral;
    if(DELTAT==0) {
        delta = (diff[1]-diff[0])/time;
        //integral -= old[old_cnt];
        //old[old_cnt] = (diff[0]+diff[1])/2.0f*0.004;
        /*old[old_cnt] = (diff[0]+diff[1])/2.0*delta;
        integral+=old[old_cnt++];*/
        integral+=(diff[0]+diff[1])/2.0*time;
    } else {
        delta = (diff[1]-diff[0])/DELTAT;
        //integral -= old[old_cnt];
        //old[old_cnt] = (diff[0]+diff[1])/2.0f*0.004;
       /* old[old_cnt] = (diff[0]+diff[1])/2.0*DELTAT;
        integral+=old[old_cnt++]; */  
        integral+=(diff[0]+diff[1])/2.0*DELTAT;   
    }

    // 積分値のオーバーを防ぐ
    if (integral>11.0) 
        integral=11.0;
    if (integral<-11.0) 
        integral=-11.0;

    if (debug) {
        sprintf(buf,"pid:(%3.1f-%3.1f), diff:%4.2f d:%4.2f i:%4.2f",target,value,diff[1],delta,integral);
        msg_f(buf,11);
    }

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
    if(debug)
        msg_f("reset PID",1);
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