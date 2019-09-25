#ifndef HPolling_
#define HPolling_

#include "ColorSensor.h"
#include "GyroSensor.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "HColorSensor.h"

#include "HBTtask.h"
#include "Odometry.h"

typedef struct hsv_t {
    float h;
    float s;
    float v;
} HSV_T;

typedef struct rgb_f_t {
    float r;
    float g;
    float b;
} RGB_F_T;


class HPolling {
    public:
	    HPolling( HBTtask* bttask,
                    Odometry* odo,
                    //ev3api::ColorSensor& color ,
                    HColorSensor& color,
                    ev3api::GyroSensor *gyro ,
                    ev3api::TouchSensor& touch ,
                    ev3api::SonarSensor& sonar ,
                    ev3api::Motor& left ,
                    ev3api::Motor& right ,
                    ev3api::Motor& arm ,
                    ev3api::Clock& clk);
        ~HPolling();
        void run();
        int32_t sonar_value(void);

        int8_t getBrightness();
        void calcBrightnessRate();
        float getBrightnessRate();

        int16_t getAnglerVelocity();
        double getGyroAngle();
        void resetGyroAngle();

        int32_t getRightCount();
        int32_t getLeftCount();
        int32_t getArmCount();
        int32_t getSonar();

        float getVelocity();
        colorid_t getColorID();
        rgb_raw_t getRGB();

        void setColorSensorMode(int mode);

        bool isColor();
        bool isGray();
        bool isWhite();
        bool isBlack();

        void resetWheelCount();
        void setWhiteLevel();
        void setWhiteLevelC();
        void setBlackLevel();
        void setBlackLevelC();
        int8_t getWhiteLevel();
        rgb_raw_t getWhiteLevelC();
        int8_t getBlackLevel();
        rgb_raw_t getBlackLevelC();
        int32_t getClock();
        int getWhileCount();
        bool isColorBlack();
        void setSkipPolling(bool skip);
        int getVoltage();
        void getHSV(hsv_t &hsv);
        void gyroReset();
        void newGyro();

        static const int REFLECT;
        static const int COLOR;
        static const int ALL;


    private:
        HBTtask* bt;
        Odometry* mOdo;
        
        //ev3api::ColorSensor& mcolor;
        HColorSensor& mcolor;
        ev3api::GyroSensor *mgyro;
        ev3api::TouchSensor& mtouch;
        ev3api::SonarSensor& msonar;
        ev3api::Motor& mleftMotor;
        ev3api::Motor& mrightMotor;
        ev3api::Motor& marmMotor;
        ev3api::Clock& mclk;

        int brightness;
        double brightness_d;
        double brightness_rate;

        colorid_t colorid;
        rgb_raw_t raw;
        //rgb_raw_t rgb;
        RGB_F_T rgb;
        int16_t anglev;
        double angle;
        bool isTouched;
        int32_t sonar;
        int32_t leftCount;
        int32_t rightCount;
        int32_t armCount;
 	    int32_t volt;
       
        uint32_t clock;

        float leftSpeed;
        float rightSpeed;

        int mColorSensorMode;

        int white_cnt;
        int gray_cnt;
        int black_cnt;

        int WHITELEVEL;
        int BLACKLEVEL;
        rgb_raw_t WHITELEVEL_C;
        rgb_raw_t BLACKLEVEL_C;

        float mKcolor;

        bool mSkipPolling;


        void calcvelocity(int32_t lWheelCount,int32_t rWheelCount,int32_t clock);
        void color_value(colorid_t& colorid,rgb_raw_t& rgb);



};

#endif
