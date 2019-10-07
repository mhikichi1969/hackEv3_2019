#include "HLowPassFilter.h"

HLowPassFilter::HLowPassFilter(int num)
{
    avg_num=num;
    current=0;
    prev=0.0;
    mRate = 0.76;
    log = new int[num];
}
HLowPassFilter::~HLowPassFilter()
{
    delete[] log;
}
int HLowPassFilter::getAvgNum()
{
    return avg_num;
}

void HLowPassFilter::addValue(int value)
{
    log[current++]=value;
    prev = prev*mRate + value*(1-mRate);
  // prev = value;
    if(current==avg_num) current=0;
}

/*float HLowPassFilter::getFillteredValue()
{
    float sum=0;
    for(int i=0;i<avg_num;i++) {
        sum += log[i];
    }
    return sum/avg_num;
}*/

float HLowPassFilter::getFillteredValue()
{
    
    return prev;
}

void HLowPassFilter::setRate(double rate)
{
    mRate = rate;
}
