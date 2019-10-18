#include "HPolling.h"
#include "HBTtask.h"
#include "Odometry.h"
#include "util.h"

const int HPolling::REFLECT = 1;
const int HPolling::COLOR = 2;
const int HPolling::ALL = 3;

using namespace ev3api;


HPolling::HPolling( HBTtask* bttask,
                    Odometry* odo,
                   // ev3api::ColorSensor& color ,
                   HColorSensor& color,
                    ev3api::GyroSensor *gyro ,
                    ev3api::TouchSensor& touch ,
                    ev3api::SonarSensor& sonar ,
                    ev3api::Motor& left ,
                    ev3api::Motor& right ,
                    ev3api::Motor& arm ,
                    ev3api::Clock& clk)
           : bt(bttask),
            mOdo(odo),
            mcolor(color),
            mgyro(gyro),
            mtouch(touch),
            msonar(sonar),
            mleftMotor(left),
            mrightMotor(right),
            marmMotor(arm),
            mclk(clk),
            mColorSensorMode(1),
            mSkipPolling(false)
            
{
    bt->init_queue();

    WHITELEVEL=20;
    BLACKLEVEL=0;

    mKcolor=0.1689;

    white_cnt=black_cnt=gray_cnt=0;
    angle = 0;
    
}
HPolling::~HPolling()
{

}

void HPolling::run()
{
   // static colorid_t id;
   // static rgb_raw_t rgb;
   //     int st = mclk.now();
  /* char buf[20];
   static int cnt=0;
   sprintf(buf,"polling  %d",cnt++);
   msg_f(buf,1);*/
  // ev3_speaker_play_tone(NOTE_E4,100);
   
   if(mSkipPolling) return;

    isTouched = mtouch.isPressed();
    sonar = sonar_value();
    leftCount = mleftMotor.getCount();
    rightCount  = mrightMotor.getCount();

   // mOdo->backlash_cancel(&leftCount,&rightCount);

    armCount = marmMotor.getCount();
    clock = mclk.now();
    volt = ev3_battery_voltage_mV();
    
   static int color=0;
   bool cSensorOn=false;

    cSensorOn=mColorSensorMode==2?true:false; // colorモードならtrue
    //colorid=mColorSensorMode==1?(colorid_t)0:colorid;  // 輝度モードならカラーidは0, カラーモードなら維持
    
    if(!cSensorOn) { // カラーセンサーの時は輝度は読まない
         brightness_d = brightness= mcolor.getBrightness();
         rgb.r=rgb.g=rgb.b=0;
    }
    else {
       // colorid=mcolor.getColorNumber();
      //  colorid=mcolor.getColorNumberAVG();
       // mcolor.getRawColor(raw);
        mcolor.getRawColorLPF(raw);
        //受光感度の調整
       // raw.r /= 0.65; 
       // raw.b /= 0.47;

        //輝度に対する正規化
       
       rgb.r = raw.r * mKcolor;
       rgb.g = raw.g * mKcolor;
       rgb.b = raw.b * mKcolor;

       brightness_d =  0.298912 * rgb.r  +
                     0.586611 * rgb.g +
                     0.114478 * rgb.b ;

        brightness = (int)brightness_d;
      /*  char buf[256];
        sprintf(buf," R:%d,G:%d,B:%d K:%lf",  rgb.r, rgb.g, rgb.b,brightness_d);
        msg_f(buf,10);*/

       // ev3_color_sensor_get_rgb_raw(EV3_PORT_2,&raw);
    }
    calcBrightnessRate();
	//ev3_speaker_play_tone(NOTE_F4,100);

//   color_value(colorid,raw);
     static double prev_anglev=0;
     //anglev = mgyro.getAnglerVelocity();
     /* 角速度を積分して求める場合*/
     //angle += (anglev+prev_anglev)/2.0*0.004;
     //prev_anglev = anglev;

     //angle = mOdo->getGyroAngle()*180/M_PI;

    /* ジャイロ角度を取得する場合*/
    if(mgyro!=nullptr)
         angle = -(double)mgyro->getAngle();  // オドメトリと角度方向が逆（反時計回りが正とする）
     
     mOdo->setGyroAngle(angle);
    //angle = ev3_gyro_sensor_get_angle(EV3_PORT_4);

    calcvelocity(leftCount,rightCount,clock);
     mOdo->calc(leftCount,rightCount);
     
     T_SENDBUF send;
     send.light = brightness;
     send.light_rate = brightness_rate;
   //  send.color = colorid;
     send.r = rgb.r;
     send.g = rgb.g;
     send.b = rgb.b;
     
     send.gyro = anglev;
     send.angle = angle;
     send.sonar = sonar;
     send.motor_l = leftCount;
     send.motor_r = rightCount;
     send.motor_arm = armCount;
     send.cnt = clock;
     send.volt = volt;

     send.x = mOdo->getGyroX();
     send.y = mOdo->getGyroY();

     /*send.x = mOdo->getX();
     send.y = mOdo->getY();*/
     send.th = mOdo->getAngle();

     send.len = mOdo->getLength();
     send.v = mOdo->getVelocity();

    static int enq_cnt=0;
    if(enq_cnt++%10==0)
         bt->enqueue(send);

/*
    char buf[256];
    static int32_t tmp[100];
    static int tmpcnt=0;
    if(tmpcnt<10)
        tmp[tmpcnt++]=clock;
    
    if(tmpcnt==10) {
        for(int i=0;i<10;i++) {
             sprintf(buf,"clock %ld",tmp[i]);
            msg_f(buf,5+i);
        }
        tmpcnt++;
    }*/
        
}

int32_t HPolling::sonar_value(void)
{
    static uint32_t counter = 0;
    static int32_t distance;

    if (++counter == 80/4) // センサー起動周期/呼び出し周期
    {
        distance = msonar.getDistance();
        counter = 0;
    }

    return distance;
}

void HPolling::color_value(colorid_t& id,rgb_raw_t& rgb)
{
    static uint32_t counter = 0;

    if (++counter == 40/40) // センサー起動周期/呼び出し周期
    {
        id=mcolor.getColorNumber();
        mcolor.getRawColor(rgb);
        counter = 0;
    }
} 

colorid_t HPolling::getColorID() 
{
    return colorid;
}
rgb_raw_t HPolling::getRGB()
{
    return raw;
}
bool HPolling::isColor() {

    return (colorid>=2 && colorid<=5);
}

int HPolling::getWhileCount()
{
    return white_cnt;
}

bool HPolling::isGray() {
    return gray_cnt>5;
}

bool HPolling::isWhite() {
    return white_cnt>3;

}
bool HPolling::isBlack() {
    return black_cnt>5;

}

bool HPolling::isColorBlack()
{
    return colorid==COLOR_BLACK;
}

void HPolling::calcvelocity(int32_t lWheelCount,int32_t rWheelCount,int32_t clock)
{
    static uint32_t counter = 0;
    static int32_t rcnt,lcnt,lclock;

    if(++counter == 400/4)
    {
        leftSpeed = (float)(lWheelCount-lcnt)/(clock-lclock);
        rightSpeed = (float)(rWheelCount-rcnt)/(clock-lclock);

        rcnt=rightCount;
        lcnt=leftCount;
        lclock = clock;

        counter=0;
    }

}

int8_t HPolling::getBrightness(void) 
{
    return brightness;
}

void HPolling::calcBrightnessRate()
{
    double grayLevel = (WHITELEVEL+BLACKLEVEL)/2.0;

    double rate = (brightness_d-grayLevel)/(WHITELEVEL-grayLevel);
    if(rate>1.0) rate=1.0;
    if(rate<-1.0) rate=-1.0;

    brightness_rate = rate;
}

float HPolling::getBrightnessRate()
{
  /*  float grayLevel = (WHITELEVEL+BLACKLEVEL)/2.0;

    float rate = (brightness_d-grayLevel)/(WHITELEVEL-grayLevel);
    if(rate>1.0) rate=1.0;
    if(rate<-1.0) rate=-1.0;

    return rate;*/
    return brightness_rate;
}


int16_t HPolling::getAnglerVelocity()
{
    return anglev;
}

double HPolling::getGyroAngle()
{
    return angle;
}
void HPolling::resetGyroAngle()
{
    angle = 0;
    if(mgyro!=nullptr)
        mgyro->reset();
}


int32_t HPolling::getRightCount()
{
    return rightCount;
}

int32_t HPolling::getLeftCount()
{
    return leftCount;
}
int32_t HPolling::getArmCount()
{
    return armCount;
}


float HPolling::getVelocity() 
{
    return (leftSpeed+rightSpeed)/2;
}

void HPolling::setColorSensorMode(int mode)
{
    mColorSensorMode = mode;
    
}

void HPolling::resetWheelCount()
{
    mOdo->resetWheelCount();
    //mleftMotor.reset();
    //mrightMotor.reset();
    mleftMotor.setCount(10000);
    mrightMotor.setCount(10000);

}

int32_t HPolling::getSonar()
{
    return sonar;
}

void HPolling::setWhiteLevel()
{
    WHITELEVEL = brightness;
/*
    char buf[256];
    sprintf(buf,"W:%d WC:  B:",WHITELEVEL);
    msg_f(buf,1);*/
}
void HPolling::setWhiteLevelC()
{
    WHITELEVEL_C = raw;

    float br =  0.298912f * raw.r  +
                0.586611f * raw.g +
                0.114478f * raw.b;

    mKcolor = WHITELEVEL/br;


  /*  char buf[256];
    sprintf(buf,"W:%d WC:%d,%d,%d B:%f",WHITELEVEL,
                                         WHITELEVEL_C.r,
                                        WHITELEVEL_C.g,
                                        WHITELEVEL_C.b,mKcolor);
    msg_f(buf,11);*/
}

void HPolling::setBlackLevel()
{
    BLACKLEVEL = brightness;

  /*  char buf[256];
    sprintf(buf,"W:%d  WC:%d,%d,%d  B:%d",WHITELEVEL,   
                                                 WHITELEVEL_C.r,
                                                WHITELEVEL_C.g,
                                                WHITELEVEL_C.b,
                                                BLACKLEVEL);
    msg_f(buf,1);  */
}

void HPolling::setBlackLevelC()
{
    BLACKLEVEL_C = raw;

  /*  char buf[256];
    sprintf(buf,"W:%d  WC:%d,%d,%d  B:%d  BC:%d,%d,%d",WHITELEVEL,
                                                WHITELEVEL_C.r,
                                                WHITELEVEL_C.g,
                                                WHITELEVEL_C.b,
                                                BLACKLEVEL,
                                                BLACKLEVEL_C.r,
                                                BLACKLEVEL_C.g,
                                               BLACKLEVEL_C.b);
    msg_f(buf,1);  */
}


int8_t HPolling::getWhiteLevel()
{
    return WHITELEVEL;
}
int8_t HPolling::getBlackLevel()
{
    return BLACKLEVEL;
}

rgb_raw_t HPolling::getWhiteLevelC()
{
    return WHITELEVEL_C;
}
rgb_raw_t HPolling::getBlackLevelC()
{
    return BLACKLEVEL_C;
}

int32_t HPolling::getClock()
{
    return clock;
}
void HPolling::setSkipPolling(bool skip)
{
    mSkipPolling = skip;
}
int HPolling::getVoltage()
{
    return volt;
}

void HPolling::gyroReset()
{
    if(mgyro!=nullptr) {
        mgyro->reset();
        mgyro->setOffset(0);
    }
}

void HPolling::newGyro()
{
    if(mgyro!=nullptr) {
        delete mgyro;
    } 
    mgyro = new GyroSensor(PORT_4);
    gyroReset();

}
void HPolling::getHSV(hsv_t& hsv)
{
    float r = rgb.r;
    float g = rgb.g;
    float b = rgb.b;
               
    float h=0, s=0, v=0;
    /*
    hsv.h = h;
    hsv.s = s;
    hsv.v = v;

    if(r<3.0 && g<3.0 && g<3.0) 
      return;*/
    
    if (r >= g && g >= b) { 
        
        if(r!=0)
            s = (r - b) / r;
        else 
            s=0;
        v = r;
        if(r==b)
            h=0;
        else if(s!=0)
            h = 60.0 * (g - b) / (r- b);
    }
    if (r >= b && b >= g)
    {
        v = r;
        if(r!=0)
          s = (r - g) / r;
        else 
            s=0;
        if(r==g)
            h=0;
        else if(s!=0)
            h = 60.0 * (g - b) / (r - g) + 360.0;
    }
    if (g >= r && r >= b)
    {
        v = g;
        if(g!=0)
            s = (g - b) / g;
        else 
            s=0;
        if(g==b)
            h=0;
        else if(s!=0)
            h = 60.0 * (b - r) / (g - b) + 120.0;
    }
    if (g >= b && b >= r)
    {
        v = g;
        if(g!=0)
            s = (g - r) / g;
        else 
            s=0;
        if(r==g)
            h=0;
        else if(s!=0)
            h = 60.0 * (b - r) / (g - r) + 120.0;
    }
    if (b >= r && r >= g)
    {
        v = b;
        if(b!=0)
            s = (b - g) / b;
        else
            s=0;
        if(g==b)
            h=0;
        else if(s!=0)
            h = 60.0 * (r - g) / (b - g) + 240.0;
    }
    if (b >= g && g >= r)
    {
        v = b;
        if(b!=0) 
            s = (b - r) / b;
        else
            s=0;
        
        if(r==b)
            h=0;
        else if(s!=0)
            h = 60.0 * (r - g) / (b - r) + 240.0;
    }

    hsv.h = h;
    hsv.s = s;
    hsv.v = v;

   /* char buf[256];
    sprintf(buf," R:%3.1f,G:%3.1f,B:%3.1f ", r,g,b);
    msg_f(buf,10);*/
}

