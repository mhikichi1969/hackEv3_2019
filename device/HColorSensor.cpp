#include "HColorSensor.h"
#include "HLowPassFilter.h"

#include "util.h"



HColorSensor::HColorSensor(ePortS port) : ColorSensor(port)
{
    lpf = new HLowPassFilter(4);
    lpf_r = new HLowPassFilter(4);
    lpf_g = new HLowPassFilter(4);
    lpf_b = new HLowPassFilter(4);

    for(int i=0;i<8;i++) {
        mCount[i]=0;
    }
    for(int i=0;i<maxlogcnt;i++) {
        log[i]=-1;
    }
    logcnt=0;
    graycnt=0;
}

int8_t  HColorSensor::getBrightness()
{
    int8_t  value = base::getBrightness();
    lpf->addValue(value);

    return value;
}


float HColorSensor::getLowPassBrightness()
{
    getBrightness();
    return lpf->getFillteredValue();
}

void HColorSensor::getRawColor(rgb_raw_t &rgb)
{
    base::getRawColor(rgb);

    lpf_r->addValue(rgb.r);
    lpf_g->addValue(rgb.g);
    lpf_b->addValue(rgb.b);
}

void HColorSensor::getRawColorLPF(rgb_raw_t &rgb)
{
    getRawColor(rgb);
    
    rgb.r = lpf_r->getFillteredValue();
    rgb.g = lpf_g->getFillteredValue();
    rgb.b = lpf_b->getFillteredValue();
}


colorid_t HColorSensor::getColorNumber()
{
    colorid_t  value = base::getColorNumber();
 /*   if(log[logcnt]!=-1)
        mCount[log[logcnt]]--;
    log[logcnt++]=value;
    mCount[value]++;
    if(logcnt==maxlogcnt) logcnt=0;*/
    return value;
}

colorid_t HColorSensor::getColorNumberAVG()
{
    int max=0;
    int idx=0;
    for(int i=0;i<8;i++) {
        if(max<mCount[i]) {
            max=mCount[i];
            idx=i;
        }
    }

    char buf[256];
    sprintf(buf,"col:%d,%d",idx,max);
    msg_f(buf,14);
    if( max>maxlogcnt/2 ) 
        return (colorid_t)idx;
    else 
        return (colorid_t)0;
}
