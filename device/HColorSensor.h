#ifndef H_COLOR_SENSOR_
#define H_COLOR_SENSOR_

#include "ev3api.h"
#include "ColorSensor.h"
#include "Port.h"
#include "HLowPassFilter.h"

using ev3api::ColorSensor;

class HColorSensor : public ColorSensor 
{
    typedef ColorSensor base;

    public:
        HColorSensor(ePortS);
        int8_t  getBrightness();
        float getLowPassBrightness();
        colorid_t getColorNumber();
        colorid_t getColorNumberAVG();
        void getRawColor(rgb_raw_t &rgb);
        void getRawColorLPF(rgb_raw_t &rgb);


    private:
        HLowPassFilter *lpf;
        HLowPassFilter *lpf_r;
        HLowPassFilter *lpf_g;
        HLowPassFilter *lpf_b;

        int mCount[8];
        static const int maxlogcnt=2;
        int log[maxlogcnt];
        int logcnt=0;

        int graycnt;
};

#endif
