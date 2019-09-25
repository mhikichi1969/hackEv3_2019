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
#include "Const.h"

enum SPEED_SEC_CMD
{ 
  LINE_,
  STRAIGHT_,
  TURN_,
  VIRTUAL_,
  PARAM_END = 99999
};

typedef struct _PARAM {
    SPEED_SEC_CMD cmd;
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

      CParam *mParam[2][10] = {{para1_l,slow_l,highvolt_l,scan,carry,vtrace,motor},   // Lコース用
                              {para1_r,slow_r,highvolt_r,scan,carry,vtrace,motor}};   // Rコース用
      int max_param = 6;

         /***************************/
         /** Rコース パラメータ１   **/
         /***************************/

      CParam para1_r[50] = {

         /** Rコース  **/
         
         {LINE_,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  138.0},  //右
          {LINE_,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線
          
          {LINE_,50, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右
          {LINE_,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速
          
          {LINE_,43, 0.0,   kp, ki, kd, 1.6,1.0,  325.0},  //右 S
          {LINE_,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_,45, 0.0, kp, ki, kd , 0.65 ,1.0,  473.0},  //左大

          {LINE_,46, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {LINE_,50, 0.0,    kp*1.8, ki*1.8, kd*1.8  , 0.8 ,1.0,  581.0},  // 左
        {LINE_,50, 0.0,    kp, ki, kd   , 1.0,1.0,  650.0},  //直線  バックストレート
         {LINE_,45, 0.0,    kp, ki, kd  , 1.0,1.0,  665.0},  //直線  バックストレート　減速
       {LINE_,45, 0.0,     kp, ki, kd  , 1/1.2,1.0,  720.0},  //左
          {LINE_,50, 0.0,     kp, ki, kd , 1.0,1.0,  753.0},  //
          {LINE_,45, 0.0,    kp, ki, kd , 1/1.3,1.0,  775.0},  //左
          {LINE_,45, 0.0,      kp*1.2, ki*1.4, kd*1.1  , 1.0,1.0,  785.0},  

          {LINE_,47, 0.0,     kp, ki, kd , 1/0.68 ,1.0,  915.0},  //右
        {LINE_,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,50, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  1130.0},  // 直線
          //{LINE_,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_,35, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
        {LINE_,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };

  CParam highvolt_r[50] = {

         /** Rコース  **/
         
         {LINE_,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {LINE_,55, 0.0,     kp2, ki2, kd2 , 1.6 ,1.0,  137.0},  //右
          {LINE_,55, 0.0,     kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_,55, 0.0,    kp2, ki2, kd2,  1/0.7,1.0,  265.0},  //右
             {LINE_,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速
          {LINE_,45, 0.0,   kp2*1.5, ki2*2.0, kd2*1.5, 1.6,1.0,  325.0},  //右 S

          {LINE_,45, 0.0,   kp2*1.5, ki2, kd2*1.5  , 1.0,1.0,  345.0},  //

          {LINE_,45, 0.0,  kp2, ki2, kd2 , 0.65 ,1.0,  473.0},  //左大

          {LINE_,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {LINE_,50, 0.0,    kp2, ki2, kd2 , 0.8 ,1.0,  581.0},  // 左
        {LINE_,50, 0.0,     kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線  バックストレート
        {LINE_,45, 0.0,      kp2, ki2, kd2  , 1/1.2,1.0,  720.0},  //左
          {LINE_,45, 0.0,      kp2, ki2, kd2 , 1.0,1.0,  753.0},  //
          {LINE_,45, 0.0,     kp2, ki2, kd2 , 1/1.3,1.0,  775.0},  //左
          {LINE_,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0  , 1.0,1.0,  785.0},  
          {LINE_,50, 0.0,      kp2, ki2, kd2 , 1/0.68 ,1.0,  915.0},  //右
        {LINE_,50, 0.0,     kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          //{LINE_,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_,30, 0.0,    kp2*0.5, ki2*0.5, kd2*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {LINE_,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };      

  CParam slow_r[50] = {

         /** Rコース  **/
         
         {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {LINE_,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.6 ,1.0,  137.0},  //右
          {LINE_,SLOW_SPEED, 0.0,     kp3, ki3, kd3, 1.0,1.0,  193.0},  //直線
          {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3,  1/0.7,1.0,  265.0},  //右
             {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.0,1.0,  275.0},  //直線 減速
          {LINE_,SLOW_SPEED, 0.0,   kp3*1.5, ki3*2.0, kd3*1.5, 1.6,1.0,  325.0},  //右 S

          {LINE_,SLOW_SPEED, 0.0,   kp3*1.5, ki3, kd3*1.5  , 1.0,1.0,  345.0},  //

          {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 0.65 ,1.0,  473.0},  //左大

          {LINE_,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 0.8 ,1.0,  581.0},  // 左
        {LINE_,SLOW_SPEED, 0.0,     kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線  バックストレート
        {LINE_,SLOW_SPEED, 0.0,      kp3, ki3, kd3  , 1/1.2,1.0,  720.0},  //左
          {LINE_,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1.0,1.0,  753.0},  //
          {LINE_,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1/1.3,1.0,  775.0},  //左
          {LINE_,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0  , 1.0,1.0,  785.0},  
          {LINE_,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1/0.68 ,1.0,  915.0},  //右
        {LINE_,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,50, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          //{LINE_,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_,30, 0.0,    kp3, ki3, kd3 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {LINE_,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };      

         /***************************/
         /** Lコース パラメータ１   **/
         /***************************/
         CParam para1_l[50] = {
         {LINE_,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_,55, 0.0,   kp, ki, kd, 0.625,1.0,  138.0},  //左
          {LINE_,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {LINE_,50, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左
          {LINE_,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_,43, 0.0,      kp, ki, kd   , 0.55,1.0,  325.0},  //左 S
          {LINE_,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_,45, 0.0,   kp, ki, kd  , 1.53,1.0,  473.0},  //右大

          {LINE_,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {LINE_,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0},  // 右
        {LINE_,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0},  //直線 バックストレート
        {LINE_,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート 減速
        {LINE_,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
          {LINE_,50, 0.0,  kp, ki, kd  , 1.0,1.0,  753.0},  //
          {LINE_,45, 0.0,     kp, ki, kd, 1.3,1.0,  775.0},  //右
          {LINE_,45, 0.0,       kp*1.2, ki*1.4, kd*1.1, 1.0,1.0,  785.0},  

          {LINE_,47, 0.0,  kp, ki, kd  , 0.68 ,1.0,  915.0},  //左
        {LINE_,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,50, 0.0,    kp, ki, kd , 1.0,1.0,  1130.0},  // 直線
          {LINE_,30, 0.0,   kp, ki, kd , 0.68,1.0,  1170.0},  //左
        {LINE_,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

         CParam highvolt_l[50] = {
          // {LINE_,20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
          // {LINE_,60, 0.0,  kp, 0.5 , kd , 1.0,1.0,  200000.0},  //直線調整用
         // {LINE_,48, 0.0,   0, 0, 0  , 1.5,1.0,  20000.0},  //右大

         /** Lコース **/
         {LINE_,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {LINE_,55, 0.0,  kp2, ki2, kd2, 0.625,1.0,  137.0},  //左
          {LINE_,55, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_,55, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {LINE_,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_,45, 0.0,     kp2*1.5, ki2*2.0, kd2*1.5  , 0.625,1.0,  325.0},  //左 S

          {LINE_,45, 0.0,    kp2*1.5, ki2, kd2*1.5 , 1.0,1.0,  345.0},  //

          {LINE_,50, 0.0,  kp2, ki2, kd2  , 1.53,1.0,  473.0},  //右大

          {LINE_,50, 0.0, kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {LINE_,50, 0.0,   kp2, ki2, kd2 , 1.25,1.0,  581.0},  // 右
        {LINE_,50, 0.0,  kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線 バックストレート
        {LINE_,45, 0.0,    kp2, ki2, kd2  , 1.2,1.0,  720.0},  //右
          {LINE_,45, 0.0,    kp2, ki2, kd2   , 1.0,1.0,  753.0},  //
          {LINE_,45, 0.0,    kp2, ki2, kd2, 1.3,1.0,  775.0},  //右
          {LINE_,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0 , 1.0,1.0,  785.0},  

          {LINE_,50, 0.0,   kp2, ki2, kd2  , 0.68 ,1.0,  915.0},  //左
        {LINE_,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          {LINE_,30, 0.0,  kp2*0.5, ki2*0.5, kd2*0.5 , 0.68,1.0,  1170.0},  //左
        {LINE_,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

         CParam slow_l[50] = {

         /** Lコース **/
         {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3, 0.625,1.0,  137.0},  //左
          {LINE_,SLOW_SPEED, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_,SLOW_SPEED, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {LINE_,SLOW_SPEED, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_,SLOW_SPEED, 0.0,     kp3*1.5, ki3*2.0, kd3*1.5  , 0.625,1.0,  325.0},  //左 S

          {LINE_,SLOW_SPEED, 0.0,    kp3*1.5, ki3, kd3*1.5 , 1.0,1.0,  345.0},  //

          {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.53,1.0,  473.0},  //右大

          {LINE_,SLOW_SPEED, 0.0, kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {LINE_,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.25,1.0,  581.0},  // 右
        {LINE_,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線 バックストレート
        {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.2,1.0,  720.0},  //右
          {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3   , 1.0,1.0,  753.0},  //
          {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3, 1.3,1.0,  775.0},  //右
          {LINE_,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0 , 1.0,1.0,  785.0},  

          {LINE_,SLOW_SPEED, 0.0,   kp3, ki3, kd3  , 0.68 ,1.0,  915.0},  //左
        {LINE_,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          {LINE_,30, 0.0,  kp3, ki3, kd3 , 0.68,1.0,  1170.0},  //左
        {LINE_,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };


        CParam sc1[50] = {
         /** Lコース **/
          {LINE_,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_,55, 0.0,   kp, ki, kd, 0.625,1.0,  137.0},  //左
          {LINE_,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線
          {LINE_,55, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左

          {LINE_,55, 0.0,     kp, ki, kd, 1.0,1.0,  270.0},  //直線
          {LINE_,50, 0.0,    kp, ki, kd  , 1.0,1.0,  276.0},  //直線 減速

          {LINE_,45, 0.0,     kp*1.5, ki*2.0, kd*1.5  , 0.625,1.0,  328.0},  //左 S

          {LINE_,45, 0.0,   kp*1.5, ki, kd*1.5 , 1.0,1.0,  345.0},  //

          {LINE_,50, 0.0,   kp, ki, kd  , 1.53,1.0,  473.0},  //右大

          {LINE_,50, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
          {LINE_,50, 0.0,   kp, ki, kd , 1.25,1.0,  581.0},  // 右
         {LINE_,50, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート
         {LINE_,50, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
          {LINE_,50, 0.0,    kp, ki, kd   , 1.0,1.0,  753.0},  //

          {VIRTUAL_,40, -21.5,  10, 0.0, 1.5 , 1.0,1.0,  820.0},  
     

         // ゴール後
        {LINE_,65, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  1130.0},  // 直線
          {LINE_,40, 0.0,   kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  1170.0},  //左
        {LINE_,23, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

        CParam scan[50] = {
           //{LINE_,20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
           {STRAIGHT_,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
          {STRAIGHT_,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {PARAM_END,0,0,0,0,0,0,0}
       };
       // 運搬テスト
        CParam carry[50] = {
//{LINE_,23, 0.0,  8, 3.05, 1.2 , 1.0,1.0,  200000.0},  
          {LINE_,23, 0.0, CARRY_KP, CARRY_KI, CARRY_KD, 1.0,1.0,  200000.0},  
          {PARAM_END,0,0,0,0,0,0,0}
       };
        CParam vtrace[50] = {
           {VIRTUAL_,40, -19, kp*1.0, ki*1.0, 5.0, 1.0,1.0,  200000.0},  
           {PARAM_END,0,0,0,0,0,0,0}
       };

       // モーターテスト
        CParam motor[50] = {
           {LINE_,100, 0.0,  0, 0, 0 , 1.0,1.0,  200000.0},  
           {PARAM_END,0,0,0,0,0,0,0}
       };



        int param_idx=0;
        int param_select;

        SimpleWalker *mWk[10];
        SimpleWalker *mActive;

};

#endif