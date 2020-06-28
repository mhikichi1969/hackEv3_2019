#ifndef _SPEED_WALKER_H_
#define _SPEED_WALKER_H_

#include "LineTracer.h"
#include "SDFile.h"
#include "Section.h"
#include "Judge.h"
#include "SectionJudge.h"
#include "Turn.h"
#include "StraightWalker.h"
#include "VirtualTracer.h"
#include "ArmControl.h"
#include "Const.h"
#include "SpeedSectionEnum.h"
#include "Flag.h"

#include "Clock.h"


typedef struct _PARAM {
    int cmd;
    double fwd;
    double target;
    double kp;
    double ki;
    double kd;
    double curve;
    double ckp;

    double len;
} CParam;

class SpeedSection : public Section {
    public:
        SpeedSection(Judge *judge,
                LineTracer *tracer,
                StraightWalker *straight,
                Turn *turn,
                VirtualTracer *vt,
                ArmControl *arm,
                SDFile *sdfile);
        bool run();
        void execUndefined();
        void execInit();
        void execTrace();
        void setCourse(int course);    
        void selectParamNo(int sel);
        void setParam(CParam tmp);

    private:
      int course=0;
        enum State {
            UNDEFINED,
            INIT,
            TRACE,
            END
        };

        State mState;
       // LineTracer *mLineTracer;
        SDFile *mSd;
        ArmControl *mArm;

      //double kd=3.5; // MS-11
      //double kp=34;
      //double ki=2.6;
      
      /* 7/29
      double kd=5.76; // MS-18
      double kp=37;
      double ki=2.6;
      */

      // max forwrd 80の場合
     /* double kd=5.51; // MS-18 6.3
      double kp=46;
      double ki=4.5;*/
      //MS-08 充電池 
      double kd=SPEED_KD; 
      double kp=SPEED_KP;
      double ki=SPEED_KI;

        double vkp=kp*30.0;
        double vki=ki*30.0;
        double vkd=kd*20.0;
        // バック仮想ライントレース用
        double vkp2=kp*6.0;
        double vki2=ki*6.00;
        double vkd2=kd*1.5; //0.16
       

      // 8400
     /* double kd=3.8; 
      double kp=31.0;
      double ki=6.4;*/

      // ハイボルテージ 9000
     /*double kd2=3.2; 
      double kp2=30.0;
      double ki2=5.0;*/
      double kd2=SPEED_KD2; 
      double kp2=SPEED_KP2;
      double ki2=SPEED_KI2;   

      double kd3=SPEED_KD_SLOW; 
      double kp3=SPEED_KP_SLOW;
      double ki3=SPEED_KI_SLOW;   

      /*************/
      /* コース選択 */
      /*************/
      CParam *mParam[2][10] = {{sc_2020,slow_l, sc_l1,sc_l2,sc_l3,sc_l5,sc_l5,experiment},   // Lコース用
                              {scan2,slow_r, sc_r2,sc_r3, sc_r4, sc_r2,sc_r3, sc_r4,experiment}};   // Rコース用
      int max_param = 8;

         /***************************/
         /** Rコース パラメータ１   **/
         /***************************/

      CParam para1_r[50] = {
         /** Rコース  **/
                   
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.4, ki*0.4, kd*0.4 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線

         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  141.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線
          
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速
          
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd, 1.61,1.0,  326.0},  //右 S
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  353.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0, kp, ki, kd , 0.65 ,1.0,  473.0},  //左大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,46, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp*1.8, ki*1.8, kd*1.8  , 0.8 ,1.0,  581.0},  // 左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd   , 1.0,1.0,  650.0},  //直線  バックストレート
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  665.0},  //直線  バックストレート　減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp, ki, kd  , 1/1.2,1.0,  720.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,     kp, ki, kd , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd , 1/1.3,1.0,  775.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,      kp*1.2, ki*1.4, kd*1.1  , 1.0,1.0,  785.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,47, 0.0,     kp, ki, kd , 1/0.68 ,1.0,  915.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  1130.0},  // 直線
          //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,35, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {(int)SPEED_SEC_CMD::PARAM_END, 0,0,0,0,0,0,0}
        };

  CParam highvolt_r[50] = {

         /** Rコース  **/
         
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,     kp2, ki2, kd2 , 1.6 ,1.0,  137.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,     kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp2, ki2, kd2,  1/0.7,1.0,  265.0},  //右
             {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp2*1.5, ki2*2.0, kd2*1.5, 1.6,1.0,  325.0},  //右 S

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp2*1.5, ki2, kd2*1.5  , 1.0,1.0,  345.0},  //

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,  kp2, ki2, kd2 , 0.65 ,1.0,  473.0},  //左大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp2, ki2, kd2 , 0.8 ,1.0,  581.0},  // 左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,     kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線  バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,      kp2, ki2, kd2  , 1/1.2,1.0,  720.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,      kp2, ki2, kd2 , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp2, ki2, kd2 , 1/1.3,1.0,  775.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0  , 1.0,1.0,  785.0},  
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,      kp2, ki2, kd2 , 1/0.68 ,1.0,  915.0},  //右
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,     kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,    kp2*0.5, ki2*0.5, kd2*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {(int)SPEED_SEC_CMD::PARAM_END, 0,0,0,0,0,0,0}
        };      

  CParam slow_r[50] = {

         /** Rコース  **/
         
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.6 ,1.0,  137.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3, 1.0,1.0,  193.0},  //直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3,  1/0.7,1.0,  265.0},  //右
             {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.0,1.0,  275.0},  //直線 減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3*1.5, ki3*2.0, kd3*1.5, 1.6,1.0,  325.0},  //右 S

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3*1.5, ki3, kd3*1.5  , 1.0,1.0,  345.0},  //

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 0.65 ,1.0,  473.0},  //左大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 0.8 ,1.0,  581.0},  // 左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線  バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3  , 1/1.2,1.0,  720.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1/1.3,1.0,  775.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0  , 1.0,1.0,  785.0},  
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1/0.68 ,1.0,  915.0},  //右
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,    kp3, ki3, kd3 , 1/0.68,1.0,  1180.0},  //右
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {(int)SPEED_SEC_CMD::PARAM_END, 0,0,0,0,0,0,0}
        };      

         /***************************/
         /** Lコース パラメータ１   **/
         /***************************/
         CParam para1_l[50] = {

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.4, ki*0.4, kd*0.4 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線

         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,   kp, ki, kd, 0.58,1.0,  139.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,      kp, ki, kd   , 0.623,1.0,  329.0},  //左 S
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  349.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.539,1.0,  473.0},  //右大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0},  // 右
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0},  //直線 バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート 減速
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,  kp, ki, kd  , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp, ki, kd, 1.3,1.0,  775.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,       kp*1.2, ki*1.4, kd*1.1, 1.0,1.0,  785.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0,  915.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  1130.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,   kp, ki, kd , 0.68,1.0,  1170.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

         CParam highvolt_l[50] = {
          // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
          // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,60, 0.0,  kp, 0.5 , kd , 1.0,1.0,  200000.0},  //直線調整用
         // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,48, 0.0,   0, 0, 0  , 1.5,1.0,  20000.0},  //右大

         /** Lコース **/
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp2, ki2, kd2, 0.625,1.0,  137.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp2*1.5, ki2*2.0, kd2*1.5  , 0.625,1.0,  325.0},  //左 S

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2*1.5, ki2, kd2*1.5 , 1.0,1.0,  345.0},  //

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,  kp2, ki2, kd2  , 1.53,1.0,  473.0},  //右大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0, kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.25,1.0,  581.0},  // 右
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,  kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線 バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.2,1.0,  720.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2, ki2, kd2   , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp2, ki2, kd2, 1.3,1.0,  775.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0 , 1.0,1.0,  785.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp2, ki2, kd2  , 0.68 ,1.0,  915.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp2*0.5, ki2*0.5, kd2*0.5 , 0.68,1.0,  1170.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

         CParam slow_l[50] = {

         /** Lコース **/
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3, 0.625,1.0,  137.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,     kp3*1.5, ki3*2.0, kd3*1.5  , 0.625,1.0,  325.0},  //左 S

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3*1.5, ki3, kd3*1.5 , 1.0,1.0,  345.0},  //

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.53,1.0,  473.0},  //右大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0, kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.25,1.0,  581.0},  // 右
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線 バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.2,1.0,  720.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3   , 1.0,1.0,  753.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3, 1.3,1.0,  775.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0 , 1.0,1.0,  785.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3  , 0.68 ,1.0,  915.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp3, ki3, kd3 , 0.68,1.0,  1170.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };


        /*************************/
        /* Lコース                */ 
        /* ショートカット無し ライントレース無し れんこん */
        /*************************/
        CParam sc_l1[50] = {
         /** Lコース **/        

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1.0,1.0,  30.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1/1.6 ,1.0,  139.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd,  0.7,1.0,  260.0},  //左

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,50, -28,  vkp, vki, vkd  , 1.0,1.0,  -1},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -28,  vkp, vki, vkd  , 1.0,1.0,  65},  
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+13},  

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, +28,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE_L},  //LEN  65+10+109
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+13+109+14}, 
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, +48, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE_L},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  72},  //直線  バックストレート

          //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, +48,  vkp, vki, vkd  , 1.0,1.0, LAST_CURVEIN_ANGLE_L },  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,55, 0.0,  0,0, 0 , 1.0,1.0,  18},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -30,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE_L},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  
          //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  155},  // 直線
                            //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99990.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  155+50},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0, 155+50 +5},  // 直線
          {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

        /* ショートカット試し用 未使用*/
        CParam sc_l1_org[50] = {
            /** Lコース **/
            /*{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,42, 0.0,  kp, ki, kd, 1.0,1.0,  70.0}, 
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,42, -21.5,  kp, ki, kd  , 1.0,1.0,  137.5},  
              {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0}, */ 


            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,      kp, ki, kd   , 0.55,1.0,  325.0},  //左 S
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.55,1.0,  473.0},  //右大

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0},  // 右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0},  //直線 バックストレート
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート 減速
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,  kp, ki, kd  , 1.0,1.0,  740.0},  // 直線真ん中

            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,40, 0.0,  0,0, 0 , 1.0,1.0,  770.0},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,35, -20,  kp, ki, kd  , 1.0,1.0,  836},  


            // ゴール後
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  900.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.0,1.0,  1000.0},  // 直線
                    //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,   kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  1040.0},  //左
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,23, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1060.0},  // 直線
            {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };
        /*************************/
        /* Lコース                */ 
        /* ショートカット無し ライントレース無し れんこん */
        /*************************/
        CParam sc_l2[50] = {
         /** Rコース **/        

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1.0,1.0,  30.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1/1.6 ,1.0,  139.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd,  0.7,1.0,  260.0},  //右

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,50, -28,  vkp, vki, vkd  , 1.0,1.0,  1},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -28,  vkp, vki, vkd  , 1.0,1.0,  65},  
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+11},  

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, +28,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE_L},  //LEN  65+10+109
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+10+109+14}, 
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, +48, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE_L},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  72},  //直線  バックストレート

          //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, +48,  vkp, vki, vkd  , 1.0,1.0, LAST_CURVEIN_ANGLE_L+6},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,55, 0.0,  0,0, 0 , 1.0,1.0,  12},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -30,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE_L},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  
          //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  155},  // 直線
                            //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99990.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  155+50},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0, 155+50 +5},  // 直線
          {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };



        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ通過 */
        /*************************/
        double offset=115;
        CParam sc_l2_org[50] = {
         /** Lコース **/        
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,   kp, ki, kd, 0.58,1.0,  139.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

          // ショートカット　ロゴ通過
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,45, -18,  kp, ki, kd  , 1.0,1.0,  195},  
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  220.0},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,45, -18,  kp, ki, kd  , 1.0,1.0,  255},  
          // ショートカット終了　コース復帰
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.55,1.0,  473.0-offset},  //右大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0-offset},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0-offset},  // 右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0-offset},  //直線 バックストレート
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0-offset},  //直線 バックストレート 減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0-offset},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,  kp, ki, kd  , 1.0,1.0,  753.0-offset},  //
                            // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  9999.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp, ki, kd, 1.3,1.0,  775.0-offset},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,       kp*1.2, ki*1.4, kd*1.1, 1.0,1.0,  785.0-offset},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0,  915.0-offset},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0-offset},  // 直線

          // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  1130.0-offset},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  1170.0-offset},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0-offset},  // 直線
          {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ通過2 れんこん*/
        /* 未使用 */
        /*************************/
        double checkpoint=259;
        CParam sc_l3[60] = {
         /** Lコース **/        
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1.0,1.0,  50.0},  // 直線
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  139.0},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

        // ショートカット　ロゴ通過
        //   {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,42, -18,  vkp, vki, vkd  , 1.0,1.0,  200.0},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
        {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  218.0},  
        //   {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,42, -18,  vkp, vki, vkd  , 1.0,1.0,  259},
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,49, -22,  vkp, vki, vkd  , 1.0,1.0,  -165},

        // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,42, 28,  vkp, vki, vkd  , 1.0,1.0,  259+90},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,49, 35,  vkp, vki, vkd  , 1.0,1.0,  -5},  
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},

        //  {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  8},  
        // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,55, 49, vkp, vki, vkd  , 1.0,1.0,  20+80},  
        //  {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 50, vkp, vki, vkd  , 1.0,1.0,  85}, 

        // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_ANG2,48, 0,  kp, ki, kd , 1.3,1.0,  45},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, 46, vkp, vki, vkd  , 1.0,1.0,  45},  //れんこん
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_ANG2,48, 0,  kp, ki, kd , 1.3,1.0,  88},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, 46, vkp, vki, vkd  , 1.0,1.0,  89},  //れんこん

        // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp*0.95, ki, kd*1.0   , 1.0,1.0,  115},  //直線  バックストレート
        {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,55, 0.0,   0,0,0   , 1.0,1.0,  115},  //直線  バックストレート
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 48,  vkp, vki, vkd  , 1.0,1.0,  90+90}, 
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, 26,  vkp, vki, vkd  , 1.0,1.0,  18},   
        {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  50},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,48, -29,  vkp, vki, vkd  , 1.0,1.0,  90-85},  

        //  {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  120},  
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  50},  

        /*
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_ANG,52, 0.0,    kp*1.05, ki*3.0, kd*1.01   , 1.0,1.0,  10},  //直線  バックストレート
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_ANG2,52, 0.0,   kp, ki*2.0, kd   , 1.2,1.0,  88},  //
        */

        //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,55, 48,  vkp, vki, vkd  , 1.0,1.0,  20+80+70+100},  
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,52, 26,  vkp, vki, vkd  , 1.0,1.0,  20},  

        // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,52, 0.0,  0,0, 0 , 1.0,1.0,  35 },  
        // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, 25,  vkp, vki, vkd  , 1.0,1.0, 115}, 
        //    {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        /*
        {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,52, 0.0,  0,0, 0 , 1.0,1.0,  56},  

        //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -29,  vkp, vki, vkd  , 1.0,1.0,  30+110},  
        {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, -28,  vkp, vki, vkd  , 1.0,1.0,  -80},  
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},

        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  60},  
        {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},*/

        // ゴール後

        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  150+50},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+5},  // 直線
        {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };


        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ往復 */
        /*************************/
        
          CParam sc_l4[50] = {
         /** Lコース **/        

         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  139.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

          // ショートカット　ロゴ通過
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  18.0},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,49, -22,  vkp, vki, vkd  , 1.0,1.0,  -145}, 
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,30, 0.0,  0,0, 0 , 1.0,1.0,  35.0},  

          //ターン ロゴ通過 戻り
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,30, 0, 0,0,0, 0,0, -145+30},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,40, 0, 0,0,0, 0,0, -145+100},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,20, 0, 0,0,0, 0,0, -145+180},


          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  27.0},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 18, vkp, vki, vkd , 1.0,1.0, 90}, 
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  10},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -18,  vkp, vki, vkd  , 1.0,1.0, 25},
          //通過完了
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,53, 0.0,  0,0, 0 , 1.0,1.0,   2},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,45, 28,  vkp, vki, vkd  , 1.0,1.0, 212 },
         // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,45, 80,  vkp, vki, vkd  , 1.0,1.0,  280.0+15.0+33+22+30+5+90+50},
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,53, 0.0,  0,0, 0 , 1.0,1.0,  52},  

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -30,  vkp, vki, vkd  , 1.0,1.0, 10 },  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp*0.5, ki*0, kd*1.2   , 1.0,1.0,  60},  

 //         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,  kp, ki, kd  , 1.0,1.0,  280.0+15.0+33+22+30+5+88+15+50},  
 //         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0, 280.0+15.0+35+22+30+5+88+15+50+180},  //左


    
         // ゴール後
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp*0.5, ki*0, kd*1.2   , 1.0,1.0,  60},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  150+50},  //左
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+10},  // 直線
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };



        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ往復 */
        /*   バック走法（本命）　　　　*/
        /*************************/
        
        CParam sc_l5[50] = {
         /** Lコース **/        
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  139.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd, 1.0,1.0,  167.0},  //直線

          // ショートカット　ロゴ通過
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  21.0},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,49, -24,  vkp, vki, vkd  , 1.0,1.0,  -145}, 
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
                 //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-40, 0.0,  0,0, 0 , 1.0,1.0,  5.0}, //強制減速

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,30, 0.0,  0,0, 0 , 1.0,1.0,  23.0}, //15  

          // ロゴ通過 バック戻り
       //   {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::ARM_, -50, 0.0,  0,0, 0 , 0.0,0.0,  0.0}, 
                             {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-85, 0.0,  0,0, 0 , 1.0,1.0,  20.0}, // 減速   
                             {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-40, 0.0,  0,0, 0 , 1.0,1.0,  10.0}, // 減速   
                           // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_ALL,0, 0.0,  0,0, 0 , 0.0,0.0,  0.0},  
          {(int)SPEED_SEC_CMD::ARM_, 0, 0.0,  0,0, 0 , 0.0,0.0,  0.0}, 

         // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-51, 0.0,  0,0, 0 , 1.0,1.0,  10.0},  //急ブレーキ
  
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-40, 0.0,  0,0, 0 , 1.0,1.0,  3.0}, 
 
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,-40, -21, vkp2, vki2, vkd2 , 1.0,1.0, -90}, 
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-49, 0.0,  0,0, 0 , 1.0,1.0,  -6},  

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,-49, 18,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_ANGLE},
          //通過完了
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-45, 0.0,  0,0, 0 , 1.0,1.0,   -8},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,-45, -27 ,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_GATE2_ANGLE },
         // {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,45, 80,  vkp, vki, vkd  , 1.0,1.0,  280.0+15.0+33+22+30+5+90+50},
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-60, 0.0,  0,0, 0 , 1.0,1.0,  -56},  

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,-54, 30.0,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_LAST_ANGLE },  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-55, 0.0,    0,0,0  , 1.0,1.0,  -45},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,30, 0, 0,0,0, 0,0, 30-180},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 0, 0,0,0, 0,0, 100-180},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,30, 0, 0,0,0, 0,0, -5},
         // ゴール後
          {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,35, 0.0,    kp*0.7, ki*0, kd*0.7   , 1.0,1.0,  60},  
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.65, ki, kd*0.7 , 0.68,1.0,  150+50},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+10},  // 直線
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

    /* ショートカットR用 未使用*/
      CParam sc_r1[50] = {

          /** Rコース  **/

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.67 ,1.0,  141.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd, 1.6,1.0,  325.0},  //右 S
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0, kp, ki, kd , 0.645 ,1.0,  473.0},  //左大

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,46, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp*1.4, ki*1.4, kd*1.4  , 0.8 ,1.0,  581.0},  // 左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd   , 1.0,1.0,  650.0},  //直線  バックストレート
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  665.0},  //直線  バックストレート　減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,     kp, ki, kd  , 1/1.2,1.0,  720.0},  //左
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,  kp, ki, kd  , 1.0,1.0,  745.0},  // 直線真ん中

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  780.0},  
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,45, 18,  kp, ki, kd  , 1.0,1.0,  840},  

          // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  900.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.0,1.0,  1000.0},  // 直線
          //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,35, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1045.0},  //右
          //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1080.0},  // 直線？
          {(int)SPEED_SEC_CMD::PARAM_END, 0,0,0,0,0,0,0}
        };

        /*************************/
        /* Rコース                */ 
        /* ショートカット無し ライントレース無し れんこん */
        /*************************/
        CParam sc_r2[50] = {
         /** Rコース **/        

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.4, ki*0.4, kd*0.4 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  141.0},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,50, 30,  vkp, vki, vkd  , 1.0,1.0,  1},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 30,  vkp, vki, vkd  , 1.0,1.0,  S1_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  8},  

            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -29,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE},  //LEN  65+10+109
                       {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
           {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  22}, 
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, -42, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  82},  //直線  バックストレート

            //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, -46,  vkp, vki, vkd  , 1.0,1.0, LAST_CURVEIN_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,55, 0.0,  0,0, 0 , 1.0,1.0,  14},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 30,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  
          //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
          {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  155},  // 直線
                            //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99990.0},  

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  155+50},  //右
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0, 155+50 +5},  // 直線
          {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

        /*************************/
        /* Rコース    本命             */ 
        /* ショートカット(ロゴ通過) ライントレース無し れんこん */
        /*************************/
        CParam sc_r3[50] = {
         /** Rコース **/        
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.4, ki*0.4, kd*0.4 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  141.0},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,49, 31,  vkp, vki, vkd  , 1.0,1.0,  1},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,49, 31,  vkp, vki, vkd  , 1.0,1.0,  S1_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  8},  

            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,48, -29,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE},  //LEN  65+10+109
                       {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
           {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  18}, 
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, -42, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  82},  //直線  バックストレート

            //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -48,  vkp, vki, vkd  , 1.0,1.0, LAST_CURVEIN_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,55, 0.0,  0,0, 0 , 1.0,1.0,  2},
            //  ロゴに侵入
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,42, 24,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE},  //25 
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  
            //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            // ゴール後
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  30},  
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  130},  // 直線
                             //      {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99990.0},  
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  150+50},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,   kp*0.5, ki*0.5, kd*0.5   , 1.0,1.0, 150+50+1},  // 直線
            {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

        /*************************/
        /* Rコース                */ 
        /* 大ショートカット ライントレース無し れんこん */
        /*************************/
        CParam sc_r4[50] = {
         /** Rコース **/        
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70, 0.0,  kp*0.4, ki*0.4, kd*0.4 , 1.0,1.0,  10.0},  // 直線
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp*0.6, ki*0.6, kd*0.6 , 1.0,1.0,  30.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  141.0},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,50, 30,  vkp, vki, vkd  , 1.0,1.0,  1},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, 30,  vkp, vki, vkd  , 1.0,1.0,  S1_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  8},  

            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,50, -29,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE},  //LEN  65+10+109
                       {(int)SPEED_SEC_CMD::RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
           {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  22}, 
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,52, -42, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  82},  //直線  バックストレート

            //{(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,55, -46,  vkp, vki, vkd  , 1.0,1.0, -180-90},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  21},
            //  ロゴ手前に侵入
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,45, 20,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

            // ゴール後
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  173},  // 直線
                   //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99990.0},  

            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  173+50},  //右
            {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0, 173+50 +5},  // 直線
            {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
        };

        CParam scan[50] = {
        {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0, 13.2, 0.16, 0.046, 1.0,1.0,  200000.0},  //低速スキャン用
       //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
       //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };
           


  // 2020テスト


    //double kp4=23,ki4= 0.6 ,kd4=1.9;   // speed 600, trk 2.5

   // double kp4=26.0,ki4= 0.35 ,kd4=3.3; // 家mac
  // double kp4=25.0,ki4= 0.31 ,kd4=3.4;  // spd 51
   //double kp4=27,ki4= 0.5 ,kd4=3.8; //spd 54ぐらいなら
   /* // 高速版ベースぱら
   double kp4=39,ki4= 1.1 ,kd4=8.75;
  double base_spd=68.0f, curce_spd=62.0f;
  */
  //超高速 SPEED 1600, FORCE 10, bodyテンソル有り 0.0001
 /* double kp4=39.0,ki4= 1.1 ,kd4=8.75;
  double base_spd=60.0f, curce_spd=55.0f;*/
    //SPEED 1300, FORCE 3 bodyテンソル無し
  double kp4=30,ki4= 5.0 ,kd4=6.0;
  double base_spd=55.0f, curce_spd=50.0f;

        CParam scan2[50] = {
                //    {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,80, 0.0,  0,0,0 , 1.0,1.0,  100000.0},  //低速スキャン用

                  //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  0,0,0 , 1.0,1.0,  20000.0},  //低速スキャン用
         // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd*0.8, 0.0,  kp4*0.8, ki4*0.8, kd4*0.8 , 1.0,1.0,  50.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  98.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd,0.0, kp4, ki4, kd4 , 0.5 ,1.0,  240.0},  //低速スキャン用
              //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  0,0,0 , 1.0,1.0,  200000.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  286.0},  //低速スキャン用
//          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  294.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,  kp4, ki4, kd4,2.75,1.0,  445.0},  //低速スキャン用

        //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  548.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  510.0},  //低速スキャン用
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd , 0.0, kp4, ki4, kd4,3.2,1.0,  585.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,    kp4, ki4, kd4, 1.0,1.0,  655.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,  kp4, ki4, kd4, 0.5,1.0,  753.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  795.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  850.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  945.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  1010.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  1220.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  1240.0},  //減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd,0.0, kp4, ki4, kd4, 3.3,1.0,  1270},  //低速スキャン用 1250.0

                        //   {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0, kp4, ki4, kd4, 1.0,1.0,  1400.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50 , 0.75, kp4, ki4, kd4, 1.0,1.0,  1460.0},  //低速スキャン用
         // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.2,kp4, ki4, kd4, 1.6,1.0,  1520.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd*0.5, 0.75,  kp4, ki4, kd4 , 1.0,1.0,  1580.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 1.0,1.0,  200000.0},  //低速スキャン用

        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,  13.0, 0.1, 0.04 , 1.0,1.0,  60.0},  //低速スキャン用
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,60, 0.0,  17, 0.1, 0.045  , 1.0,1.0,  200000.0},  //低速スキャン用
       //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
       //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };

/*
        CParam scan2[50] = {
                  //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  0,0,0 , 1.0,1.0,  20000.0},  //低速スキャン用
         // {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd*0.8, 0.0,  kp4*0.8, ki4*0.8, kd4*0.8 , 1.0,1.0,  50.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  98.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd,0.0, kp4, ki4, kd4 , 0.6 ,1.0,  240.0},  //低速スキャン用
              //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  0,0,0 , 1.0,1.0,  200000.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  286.0},  //低速スキャン用
//          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  294.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,  kp4, ki4, kd4,2.75,1.0,  454.0},  //低速スキャン用

        //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  548.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  548.0},  //低速スキャン用
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd , 0.0, kp4, ki4, kd4,3.2,1.0,  610.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,    kp4, ki4, kd4, 1.0,1.0,  670.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,  kp4, ki4, kd4, 0.5,1.0,  753.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  800.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  870.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd, 0.0,   kp4, ki4, kd4, 1.0,1.0,  968.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  1065.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd*1.1, 0.0,   kp4, ki4, kd4, 1.0,1.0,  1240.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  1280.0},  //減速
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd,0.0, kp4, ki4, kd4, 3.3,1.0,  1315},  //低速スキャン用 1250.0

                        //   {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 0.0,1.0,  200000.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd*1.1, 0.0, kp4, ki4, kd4, 1.0,1.0,  1430.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,50 , 0.75, kp4, ki4, kd4, 1.0,1.0,  1560.0},  //低速スキャン用
         // ゴール後
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd, 0.2,kp4, ki4, kd4, 1.7,1.0,  1580.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd*0.5, 0.75,  kp4, ki4, kd4 , 1.0,1.0,  1710.0},  //低速スキャン用
               //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 1.0,1.0,  200000.0},  //低速スキャン用

        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,  13.0, 0.1, 0.04 , 1.0,1.0,  60.0},  //低速スキャン用
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,60, 0.0,  17, 0.1, 0.045  , 1.0,1.0,  200000.0},  //低速スキャン用
       //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
       //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };
*/

     float sk=1.4;
    //double kp4=13.5,ki4= 1.0 ,kd4=1.7;
    /* double kp4=21.0*sk;
     double ki4= 0.45*sk;
     double kd4=2.00*sk;*/
        CParam scan3[50] = {
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  100.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0, kp4, ki4, kd4, 0.5,1.0,  238.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,  kp4, ki4, kd4, 1.0,1.0,  290.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,  kp4, ki4, kd4, 2.0,1.0,  440.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,   kp4, ki4, kd4 , 1.0,1.0,  520.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,  kp4, ki4, kd4 , 2.0,1.0,  574.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,   kp4, ki4, kd4, 1.0,1.0,  660.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,  kp4, ki4, kd4, 0.5,1.0,  740.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,  kp4, ki4, kd4 , 1.0,1.0,  770.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  855.0},  //低速スキャン用

          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,   kp4, ki4, kd4, 1.0,1.0,  935.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,   kp4, ki4, kd4 , 0.5,1.0,  1005.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,80*sk, 0.0,   kp4, ki4, kd4, 1.0,1.0,  1200.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,   kp4, ki4, kd4, 1.0,1.0,  1215.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,70*sk, 0.0,  kp4, ki4, kd4, 2.0,1.0,  1270.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,75*sk, 0.0,   kp4, ki4, kd4, 1.0,1.0,  1360.0},  //低速スキャン用
                 {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 1.0,1.0,  200000.0},  //低速スキャン用

        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,40, 0.0,  13.0, 0.1, 0.04 , 1.0,1.0,  60.0},  //低速スキャン用
        //{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,60, 0.0,  17, 0.1, 0.045  , 1.0,1.0,  200000.0},  //低速スキャン用
       //    {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
       //   {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };

double kp5=20.0,ki5= 2.0 ,kd5=0.5;
double base_spd2=46.5f, curce_spd2=41.5f;
        CParam sc_2020[50] = {
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,base_spd2, 0.0,  kp5, ki5, kd5 , 1.0,1.0,  50.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::VIRTUAL_ | (int)End::END_ANG,curce_spd2, -100,  kp5, ki5, kd5  , -5.0 ,1.0,  -89}, 
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},
         // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,base_spd2,  0, 0,0,0,  0.0,0.0,  20.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,base_spd2,  -90,  kp5*0.6, ki5*0.06, kd5*0.06,  0.0,0.0,  15.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_ | (int)End::END_ANG2,curce_spd2, -50,  kp5, ki5, kd5  , -10.0 ,1.0,  -179},  
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {(int)SPEED_SEC_CMD::ARM_, -25, 0.0,  0,0, 0 , 0.0,0.0,  0.0}, 
        //  {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,base_spd2,  0, 0,0,0,  0.0,0.0,  18.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,base_spd2*0.75,  -180, kp5*0.6, ki5*0.06 ,kd5*0.06,  0.0,0.0,  18.0},  

          //急停止
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,-base_spd2*2,  0, kp5*0.6, ki5*0.06 ,kd5*0.06,  0.0,0.0,  00.0},  
//          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-base_spd2,  0, 0,0,0,  0.0,0.0,  -95.0}, 
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,-base_spd2,  0.0 ,  kp5, ki5, kd5,  0.0,0.0,  -95.0}, 
          {(int)SPEED_SEC_CMD::ARM_, 0, 0.0,  0,0, 0 , 0.0,0.0,  0.0}, 
          {(int)SPEED_SEC_CMD::VIRTUAL_ | (int)End::END_ANG2,-curce_spd2, -80,   kp5, ki5, kd5 , 10.0 ,1.0,  -101}, 
                      {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0}, 
          //{(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,-base_spd2,  0, 0,0,0,  0.0,0.0,  -2.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,-base_spd2*0.7, -100, kp5*0.6, ki5*0.06 ,kd5*0.06,  0.0,0.0,  -3.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,base_spd2*2,  100, 0,0,0,  0.0,0.0,  0.0},  
          // 後進終了
           {(int)SPEED_SEC_CMD::VIRTUAL_ | (int)End::END_ANG2,curce_spd2, -70, kp5, ki5, kd5  , -9.0 ,1.0,  -176}, 
                      {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0}, 
//          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,base_spd2,  0, 0,0,0,  0.0,0.0,  159.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,base_spd2,  -177 ,kp5, ki5, kd5,  0.0,0.0,  165.0},  
          {(int)SPEED_SEC_CMD::VIRTUAL_ | (int)End::END_ANG2,curce_spd2, 48, kp5, ki5, kd5 , 10.0 ,1.0,  -90},  
                      {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0}, 
         // {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,base_spd2,  0, 0,0,0,  0.0,0.0,  85.0},
                    {(int)SPEED_SEC_CMD::VIRTUAL_LINE_ | (int)End::END_LEN,base_spd2,  -90,kp5*0.6, ki5*0.06 ,kd5*0.06,  0.0,0.0,  85.0},

      //ゴール
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,20,  0.5, kp5, ki5, kd5,  0.0,0.0,  125.0},  
         {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_ANG2,curce_spd2,  0, kp5, ki5, kd5,  0.0,0.0,  -10.0},  
                      {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0}, 
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,curce_spd2,  0.5, kp5, ki5, kd5,  0.0,0.0,  50.0},  

           //  {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,0, 0.0,  0,0,0 , 1.0,1.0,  20000.0},  //低速スキャン用
          {(int)SPEED_SEC_CMD::PARAM_END, 0,0,0,0,0,0,0}

        };
       // 運搬テスト
        CParam carry[50] = {
//{(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,23, 0.0,  8, 3.05, 1.2 , 1.0,1.0,  200000.0},  
          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,23, 0.0, CARRY_KP, CARRY_KI, CARRY_KD, 1.0,1.0,  200000.0},  
          {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };



       // 仮想トレーステスト
        CParam vtrace[50] = {
         //  {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_LEN,40, -19, kp*1.0, ki*1.0, 5.0, 1.0,1.0,  200000.0},
         // {SET_GOAL_PT_,0,0,0,0,0,0,0,0},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,5, 5, vkp*0.5, vki*0.5, vkd*0.5, 1.0,1.0,  90.0},

          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  


          {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,45, 0, kp*1.0, ki*1.0, kd, 1.0,1.0,  50.0}, 
          {(int)SPEED_SEC_CMD::STRAIGHT_  | (int)End::END_LEN,-45,0, 0,0,0, 0,0, 20.0} ,
       //   {(int)SPEED_SEC_CMD::(int)SPEED_SEC_CMD::TURN_,20, 0 , 0,0,0 , 1.0,1.0,  185.0},
          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,-45, -18, vkp*0.5, vki*0.5, vkd*0.5, 1.0,1.0,  180.0},

          {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,-45, 18, vkp*1.0, vki*1.0, vkd, 1.0,1.0,  -180.0},
          {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  

           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };



       // モーターテスト
        CParam motor[50] = {
           {(int)SPEED_SEC_CMD::LINE_ | (int)End::END_LEN,100, 0.0,  0, 0, 0 , 1.0,1.0,  200000.0},  
           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };

        CParam experiment[50] = {
           {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG,8, -10, vkp, vki*0, vkd, 1.0,1.0, -20},  
           {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2, 8, 10,   vkp, vki*0, vkd, 1.0,1.0,  0.0},  
            {(int)SPEED_SEC_CMD::TURN_ | (int)End::END_ANG2,-5, 0, 0,0,0, 0,0, 0},
            {(int)SPEED_SEC_CMD::RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},
           {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,20, 0.0,  vkp, vki, vkd , 1.0,1.0,  30.0},  //低速スキャン用
           {(int)SPEED_SEC_CMD::STRAIGHT_ | (int)End::END_LEN,0, 0.0,  0, 0, 0 , 1.0,1.0,  9999.0},  //低速スキャン用

           {(int)SPEED_SEC_CMD::PARAM_END,0,0,0,0,0,0,0}
       };


        int param_idx=0;
        int param_select;

        SimpleWalker *mWk[10];
        SimpleWalker *mActive;

        ev3api::Clock mclk;
        int time_cnt=0;


};

#endif