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
#include "SpeedSectionEnum.h"
#include "Flag.h"


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

        double vkp=kp*3.0;
        double vki=ki*3.0;
        double vkd=kd*3.0;
        double vkp2=kp*1.0;
        double vki2=ki*1.0;
        double vkd2=kd*0.125;
       

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
      CParam *mParam[2][10] = {{para1_l,slow_l,sc_l1,sc_l2,sc_l3,sc_l4,sc_l5},   // Lコース用
                              {para1_r,slow_r,sc_r1,sc_r2,carry,vtrace,motor}};   // Rコース用
      int max_param = 7;

         /***************************/
         /** Rコース パラメータ１   **/
         /***************************/

      CParam para1_r[50] = {
         /** Rコース  **/
         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd , 1.6 ,1.0,  138.0},  //右
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線
          
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速
          
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd, 1.6,1.0,  325.0},  //右 S
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0, kp, ki, kd , 0.65 ,1.0,  473.0},  //左大

          {LINE_ | Flag::END_LEN,46, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
          {LINE_ | Flag::END_LEN,50, 0.0,    kp*1.8, ki*1.8, kd*1.8  , 0.8 ,1.0,  581.0},  // 左
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd   , 1.0,1.0,  650.0},  //直線  バックストレート
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  665.0},  //直線  バックストレート　減速
          {LINE_ | Flag::END_LEN,45, 0.0,     kp, ki, kd  , 1/1.2,1.0,  720.0},  //左
          {LINE_ | Flag::END_LEN,50, 0.0,     kp, ki, kd , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd , 1/1.3,1.0,  775.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,      kp*1.2, ki*1.4, kd*1.1  , 1.0,1.0,  785.0},  

          {LINE_ | Flag::END_LEN,47, 0.0,     kp, ki, kd , 1/0.68 ,1.0,  915.0},  //右
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  1130.0},  // 直線
          //{LINE_ | Flag::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_ | Flag::END_LEN,35, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_ | Flag::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
        {LINE_ | Flag::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };

  CParam highvolt_r[50] = {

         /** Rコース  **/
         
         {LINE_ | Flag::END_LEN,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,     kp2, ki2, kd2 , 1.6 ,1.0,  137.0},  //右
          {LINE_ | Flag::END_LEN,55, 0.0,     kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_ | Flag::END_LEN,55, 0.0,    kp2, ki2, kd2,  1/0.7,1.0,  265.0},  //右
             {LINE_ | Flag::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速
          {LINE_ | Flag::END_LEN,45, 0.0,   kp2*1.5, ki2*2.0, kd2*1.5, 1.6,1.0,  325.0},  //右 S

          {LINE_ | Flag::END_LEN,45, 0.0,   kp2*1.5, ki2, kd2*1.5  , 1.0,1.0,  345.0},  //

          {LINE_ | Flag::END_LEN,45, 0.0,  kp2, ki2, kd2 , 0.65 ,1.0,  473.0},  //左大

          {LINE_ | Flag::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,    kp2, ki2, kd2 , 0.8 ,1.0,  581.0},  // 左
        {LINE_ | Flag::END_LEN,50, 0.0,     kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線  バックストレート
        {LINE_ | Flag::END_LEN,45, 0.0,      kp2, ki2, kd2  , 1/1.2,1.0,  720.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,      kp2, ki2, kd2 , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,     kp2, ki2, kd2 , 1/1.3,1.0,  775.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0  , 1.0,1.0,  785.0},  
          {LINE_ | Flag::END_LEN,50, 0.0,      kp2, ki2, kd2 , 1/0.68 ,1.0,  915.0},  //右
        {LINE_ | Flag::END_LEN,50, 0.0,     kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          //{LINE_ | Flag::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_ | Flag::END_LEN,30, 0.0,    kp2*0.5, ki2*0.5, kd2*0.5 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_ | Flag::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {LINE_ | Flag::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };      

  CParam slow_r[50] = {

         /** Rコース  **/
         
         {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.6 ,1.0,  137.0},  //右
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3, 1.0,1.0,  193.0},  //直線
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3,  1/0.7,1.0,  265.0},  //右
             {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.0,1.0,  275.0},  //直線 減速
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3*1.5, ki3*2.0, kd3*1.5, 1.6,1.0,  325.0},  //右 S

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3*1.5, ki3, kd3*1.5  , 1.0,1.0,  345.0},  //

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 0.65 ,1.0,  473.0},  //左大

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 0.8 ,1.0,  581.0},  // 左
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線  バックストレート
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3  , 1/1.2,1.0,  720.0},  //左
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1/1.3,1.0,  775.0},  //左
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0  , 1.0,1.0,  785.0},  
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,      kp3, ki3, kd3 , 1/0.68 ,1.0,  915.0},  //右
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          //{LINE_ | Flag::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_ | Flag::END_LEN,30, 0.0,    kp3, ki3, kd3 , 1/0.68,1.0,  1180.0},  //右
        //{LINE_ | Flag::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
          {LINE_ | Flag::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };      

         /***************************/
         /** Lコース パラメータ１   **/
         /***************************/
         CParam para1_l[50] = {
         {LINE_ | Flag::END_LEN,52, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,52, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
          {LINE_ | Flag::END_LEN,52, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {LINE_ | Flag::END_LEN,50, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_ | Flag::END_LEN,43, 0.0,      kp, ki, kd   , 0.55,1.0,  325.0},  //左 S
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.53,1.0,  473.0},  //右大

          {LINE_ | Flag::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0},  // 右
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0},  //直線 バックストレート
        {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート 減速
        {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
          {LINE_ | Flag::END_LEN,50, 0.0,  kp, ki, kd  , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,     kp, ki, kd, 1.3,1.0,  775.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,       kp*1.2, ki*1.4, kd*1.1, 1.0,1.0,  785.0},  

          {LINE_ | Flag::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0,  915.0},  //左
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  1130.0},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,   kp, ki, kd , 0.68,1.0,  1170.0},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

         CParam highvolt_l[50] = {
          // {LINE_ | Flag::END_LEN,20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
          // {LINE_ | Flag::END_LEN,60, 0.0,  kp, 0.5 , kd , 1.0,1.0,  200000.0},  //直線調整用
         // {LINE_ | Flag::END_LEN,48, 0.0,   0, 0, 0  , 1.5,1.0,  20000.0},  //右大

         /** Lコース **/
         {LINE_ | Flag::END_LEN,55, 0.0,  kp2, ki2, kd2 , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,  kp2, ki2, kd2, 0.625,1.0,  137.0},  //左
          {LINE_ | Flag::END_LEN,55, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_ | Flag::END_LEN,55, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {LINE_ | Flag::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_ | Flag::END_LEN,45, 0.0,     kp2*1.5, ki2*2.0, kd2*1.5  , 0.625,1.0,  325.0},  //左 S

          {LINE_ | Flag::END_LEN,45, 0.0,    kp2*1.5, ki2, kd2*1.5 , 1.0,1.0,  345.0},  //

          {LINE_ | Flag::END_LEN,50, 0.0,  kp2, ki2, kd2  , 1.53,1.0,  473.0},  //右大

          {LINE_ | Flag::END_LEN,50, 0.0, kp2, ki2, kd2 , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.25,1.0,  581.0},  // 右
        {LINE_ | Flag::END_LEN,50, 0.0,  kp2, ki2, kd2  , 1.0,1.0,  665.0},  //直線 バックストレート
        {LINE_ | Flag::END_LEN,45, 0.0,    kp2, ki2, kd2  , 1.2,1.0,  720.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,    kp2, ki2, kd2   , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,    kp2, ki2, kd2, 1.3,1.0,  775.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,      kp2*2.0, ki2*2.0, kd2*2.0 , 1.0,1.0,  785.0},  

          {LINE_ | Flag::END_LEN,50, 0.0,   kp2, ki2, kd2  , 0.68 ,1.0,  915.0},  //左
        {LINE_ | Flag::END_LEN,50, 0.0,   kp2, ki2, kd2 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp2, ki2, kd2 , 1.0,1.0,  1130.0},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp2*0.5, ki2*0.5, kd2*0.5 , 0.68,1.0,  1170.0},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

         CParam slow_l[50] = {

         /** Lコース **/
         {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3 , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3, 0.625,1.0,  137.0},  //左
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp2, ki2, kd2, 1.0,1.0,  193.0},  //直線
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp2, ki2, kd2,  0.7,1.0,  265.0},  //左

             {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp2, ki2, kd2  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,     kp3*1.5, ki3*2.0, kd3*1.5  , 0.625,1.0,  325.0},  //左 S

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3*1.5, ki3, kd3*1.5 , 1.0,1.0,  345.0},  //

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.53,1.0,  473.0},  //右大

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0, kp3, ki3, kd3 , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.25,1.0,  581.0},  // 右
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,  kp3, ki3, kd3  , 1.0,1.0,  665.0},  //直線 バックストレート
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3  , 1.2,1.0,  720.0},  //右
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3   , 1.0,1.0,  753.0},  //
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3, 1.3,1.0,  775.0},  //右
          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,      kp3*2.0, ki3*2.0, kd3*2.0 , 1.0,1.0,  785.0},  

          {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3  , 0.68 ,1.0,  915.0},  //左
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,   kp3, ki3, kd3 , 1.0,1.0,  980.0},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,SLOW_SPEED, 0.0,    kp3, ki3, kd3 , 1.0,1.0,  1130.0},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp3, ki3, kd3 , 0.68,1.0,  1170.0},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

        /* ショートカット試し用 */
        CParam sc_l1[50] = {
         /** Lコース **/
         /*{LINE_ | Flag::END_LEN,42, 0.0,  kp, ki, kd, 1.0,1.0,  70.0}, 
          {VIRTUAL_ | Flag::END_LEN,42, -21.5,  kp, ki, kd  , 1.0,1.0,  137.5},  
             {LINE_ | Flag::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0}, */ 
        

         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {LINE_ | Flag::END_LEN,50, 0.0,   kp, ki, kd,  0.7,1.0,  265.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速

          {LINE_ | Flag::END_LEN,43, 0.0,      kp, ki, kd   , 0.55,1.0,  325.0},  //左 S
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.55,1.0,  473.0},  //右大

          {LINE_ | Flag::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0},  // 右
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0},  //直線 バックストレート
        {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0},  //直線 バックストレート 減速
        {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,  kp, ki, kd  , 1.0,1.0,  740.0},  // 直線真ん中

           {STRAIGHT_ | Flag::END_LEN,40, 0.0,  0,0, 0 , 1.0,1.0,  770.0},  
          {VIRTUAL_ | Flag::END_LEN,35, -20,  kp, ki, kd  , 1.0,1.0,  836},  
    

         // ゴール後
        {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  900.0},  // 直線
        {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd , 1.0,1.0,  1000.0},  // 直線
                    //    {STRAIGHT_ | Flag::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  

          {LINE_ | Flag::END_LEN,40, 0.0,   kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  1040.0},  //左
        {LINE_ | Flag::END_LEN,23, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1060.0},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ通過 */
        /*************************/
        double offset=115;
        CParam sc_l2[50] = {
         /** Lコース **/        

         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

          // ショートカット　ロゴ通過
          {VIRTUAL_ | Flag::END_LEN,45, -18,  kp, ki, kd  , 1.0,1.0,  195},  
          {STRAIGHT_ | Flag::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  220.0},  
          {VIRTUAL_ | Flag::END_LEN,45, -18,  kp, ki, kd  , 1.0,1.0,  255},  
          // ショートカット終了　コース復帰
          {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.55,1.0,  473.0-offset},  //右大

          {LINE_ | Flag::END_LEN,46, 0.0,    kp, ki, kd , 1.0,1.0,  515.0-offset},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,   kp*1.8, ki*1.8, kd*1.8 , 1.25,1.0,  581.0-offset},  // 右
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd  , 1.0,1.0,  650.0-offset},  //直線 バックストレート
        {LINE_ | Flag::END_LEN,45, 0.0,   kp, ki, kd  , 1.0,1.0,  665.0-offset},  //直線 バックストレート 減速
        {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.2,1.0,  720.0-offset},  //右
          {LINE_ | Flag::END_LEN,50, 0.0,  kp, ki, kd  , 1.0,1.0,  753.0-offset},  //
                              // {STRAIGHT_ | Flag::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  9999.0},  

          {LINE_ | Flag::END_LEN,45, 0.0,     kp, ki, kd, 1.3,1.0,  775.0-offset},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,       kp*1.2, ki*1.4, kd*1.1, 1.0,1.0,  785.0-offset},  

          {LINE_ | Flag::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0,  915.0-offset},  //左
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  980.0-offset},  // 直線
    
         // ゴール後
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  1130.0-offset},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  1170.0-offset},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  1190.0-offset},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ通過2 れんこん*/
        /*************************/
        double checkpoint=259;
        CParam sc_l3[60] = {
         /** Lコース **/        
        {LINE_ | Flag::END_LEN,55, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1.0,1.0,  50.0},  // 直線
        {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
        {LINE_ | Flag::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
        {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

        // ショートカット　ロゴ通過
        //   {VIRTUAL_ | Flag::END_LEN,42, -18,  vkp, vki, vkd  , 1.0,1.0,  200.0},  
        {VIRTUAL_ | Flag::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
        {STRAIGHT_ | Flag::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  218.0},  
        //   {VIRTUAL_ | Flag::END_LEN,42, -18,  vkp, vki, vkd  , 1.0,1.0,  259},
        {VIRTUAL_ | Flag::END_ANG2,49, -22,  vkp, vki, vkd  , 1.0,1.0,  -165},

        // {VIRTUAL_ | Flag::END_LEN,42, 28,  vkp, vki, vkd  , 1.0,1.0,  259+90},  
        {VIRTUAL_ | Flag::END_ANG2,49, 35,  vkp, vki, vkd  , 1.0,1.0,  -5},  
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},

        //  {STRAIGHT_ | Flag::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  8},  
        // {VIRTUAL_ | Flag::END_LEN,55, 49, vkp, vki, vkd  , 1.0,1.0,  20+80},  
        //  {VIRTUAL_ | Flag::END_ANG2,50, 50, vkp, vki, vkd  , 1.0,1.0,  85}, 

        // {LINE_ | Flag::END_ANG2,48, 0,  kp, ki, kd , 1.3,1.0,  45},  
        {VIRTUAL_ | Flag::END_ANG2,55, 46, vkp, vki, vkd  , 1.0,1.0,  45},  //れんこん
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        //{LINE_ | Flag::END_ANG2,48, 0,  kp, ki, kd , 1.3,1.0,  88},  
        {VIRTUAL_ | Flag::END_ANG2,55, 46, vkp, vki, vkd  , 1.0,1.0,  89},  //れんこん

        // {LINE_ | Flag::END_LEN,50, 0.0,   kp*0.95, ki, kd*1.0   , 1.0,1.0,  115},  //直線  バックストレート
        {STRAIGHT_ | Flag::END_LEN,55, 0.0,   0,0,0   , 1.0,1.0,  115},  //直線  バックストレート
        {VIRTUAL_ | Flag::END_ANG2,50, 48,  vkp, vki, vkd  , 1.0,1.0,  90+90}, 
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        {VIRTUAL_ | Flag::END_LEN,50, 26,  vkp, vki, vkd  , 1.0,1.0,  18},   
        {STRAIGHT_ | Flag::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  50},  
        {VIRTUAL_ | Flag::END_ANG2,48, -29,  vkp, vki, vkd  , 1.0,1.0,  90-85},  

        //  {STRAIGHT_ | Flag::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  
        {LINE_ | Flag::END_LEN,50, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  120},  
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        {LINE_ | Flag::END_LEN,40, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  50},  

        /*
        {LINE_ | Flag::END_ANG,52, 0.0,    kp*1.05, ki*3.0, kd*1.01   , 1.0,1.0,  10},  //直線  バックストレート
        {LINE_ | Flag::END_ANG2,52, 0.0,   kp, ki*2.0, kd   , 1.2,1.0,  88},  //
        */

        //{VIRTUAL_ | Flag::END_LEN,55, 48,  vkp, vki, vkd  , 1.0,1.0,  20+80+70+100},  
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        // {VIRTUAL_ | Flag::END_LEN,52, 26,  vkp, vki, vkd  , 1.0,1.0,  20},  

        // {STRAIGHT_ | Flag::END_LEN,52, 0.0,  0,0, 0 , 1.0,1.0,  35 },  
        // {VIRTUAL_ | Flag::END_ANG2,52, 25,  vkp, vki, vkd  , 1.0,1.0, 115}, 
        //    {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},
        /*
        {STRAIGHT_ | Flag::END_LEN,52, 0.0,  0,0, 0 , 1.0,1.0,  56},  

        //{VIRTUAL_ | Flag::END_LEN,50, -29,  vkp, vki, vkd  , 1.0,1.0,  30+110},  
        {VIRTUAL_ | Flag::END_ANG2,52, -28,  vkp, vki, vkd  , 1.0,1.0,  -80},  
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},

        {LINE_ | Flag::END_LEN,50, 0.0,    kp*0.5, ki*0.0, kd*1.5   , 1.0,1.0,  60},  
        {RESET_LENGTH_, 0,0, 0,0,0, 0,0, 0},*/

        // ゴール後

        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
        {LINE_ | Flag::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  150+50},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+5},  // 直線
        {PARAM_END,0,0,0,0,0,0,0}
        };


        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ往復 */
        /*************************/
        
          CParam sc_l4[50] = {
         /** Lコース **/        

         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
          {LINE_ | Flag::END_LEN,52, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

          // ショートカット　ロゴ通過
            {VIRTUAL_ | Flag::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
            {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  18.0},  
          {VIRTUAL_ | Flag::END_ANG2,49, -22,  vkp, vki, vkd  , 1.0,1.0,  -145}, 
            {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,30, 0.0,  0,0, 0 , 1.0,1.0,  35.0},  

          //ターン ロゴ通過 戻り
          {TURN_ | Flag::END_ANG2,30, 0, 0,0,0, 0,0, -145+30},
          {TURN_ | Flag::END_ANG2,40, 0, 0,0,0, 0,0, -145+100},
          {TURN_ | Flag::END_ANG2,20, 0, 0,0,0, 0,0, -145+180},


          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  27.0},  
          {VIRTUAL_ | Flag::END_ANG2,50, 18, vkp, vki, vkd , 1.0,1.0, 90}, 
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  10},  
          {VIRTUAL_ | Flag::END_ANG2,50, -18,  vkp, vki, vkd  , 1.0,1.0, 25},
          //通過完了
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {STRAIGHT_ | Flag::END_LEN,53, 0.0,  0,0, 0 , 1.0,1.0,   2},  
          {VIRTUAL_ | Flag::END_ANG2,45, 28,  vkp, vki, vkd  , 1.0,1.0, 212 },
         // {VIRTUAL_ | Flag::END_LEN,45, 80,  vkp, vki, vkd  , 1.0,1.0,  280.0+15.0+33+22+30+5+90+50},
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {STRAIGHT_ | Flag::END_LEN,53, 0.0,  0,0, 0 , 1.0,1.0,  52},  

          {VIRTUAL_ | Flag::END_ANG2,50, -30,  vkp, vki, vkd  , 1.0,1.0, 10 },  
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {LINE_ | Flag::END_LEN,45, 0.0,    kp*0.5, ki*0, kd*1.2   , 1.0,1.0,  60},  

 //         {LINE_ | Flag::END_LEN,40, 0.0,  kp, ki, kd  , 1.0,1.0,  280.0+15.0+33+22+30+5+88+15+50},  
 //         {LINE_ | Flag::END_LEN,47, 0.0,  kp, ki, kd  , 0.68 ,1.0, 280.0+15.0+35+22+30+5+88+15+50+180},  //左


    
         // ゴール後
            {LINE_ | Flag::END_LEN,45, 0.0,    kp*0.5, ki*0, kd*1.2   , 1.0,1.0,  60},  
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  150+50},  //左
        {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+10},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };



        /*************************/
        /* Lコース                */ 
        /* ショートカット ロゴ往復 */
        /*   バック走法　　　　　　*/
        /*************************/
        
          CParam sc_l5[50] = {
         /** Lコース **/        

         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,50, 0.0,   kp, ki, kd, 0.58,1.0,  138.0},  //左
          {LINE_ | Flag::END_LEN,52, 0.0,    kp, ki, kd, 1.0,1.0,  165.0},  //直線

          // ショートカット　ロゴ通過
            {VIRTUAL_ | Flag::END_ANG,49, -18,  vkp, vki, vkd  , 1.0,1.0, -90},  
            {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,49, 0.0,  0,0, 0 , 1.0,1.0,  20.0},  
          {VIRTUAL_ | Flag::END_ANG2,49, -22,  vkp, vki, vkd  , 1.0,1.0,  -145}, 
            {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,30, 0.0,  0,0, 0 , 1.0,1.0,  18.0},  

          // ロゴ通過 バック戻り
       //   {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,-51, 0.0,  0,0, 0 , 1.0,1.0,  15.0},  //急ブレーキ
          {STRAIGHT_ | Flag::END_LEN,-40, 0.0,  0,0, 0 , 1.0,1.0,  2.0},  
          {VIRTUAL_ | Flag::END_ANG2,-40, -18, vkp2, vki2, vkd2 , 1.0,1.0, -90}, 
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {STRAIGHT_ | Flag::END_LEN,-49, 0.0,  0,0, 0 , 1.0,1.0,  -4},  

          {VIRTUAL_ | Flag::END_ANG2,-49, 18,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_ANGLE},
          //通過完了
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {STRAIGHT_ | Flag::END_LEN,-45, 0.0,  0,0, 0 , 1.0,1.0,   -8},  
          {VIRTUAL_ | Flag::END_ANG2,-45, -27,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_GATE2_ANNGEL },
         // {VIRTUAL_ | Flag::END_LEN,45, 80,  vkp, vki, vkd  , 1.0,1.0,  280.0+15.0+33+22+30+5+90+50},
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {STRAIGHT_ | Flag::END_LEN,-53, 0.0,  0,0, 0 , 1.0,1.0,  -50},  

          {VIRTUAL_ | Flag::END_ANG2,-50, 30,  vkp2, vki2, vkd2  , 1.0,1.0, BACK_LAST_ANGLE },  
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},

          {STRAIGHT_ | Flag::END_LEN,-55, 0.0,    0,0,0  , 1.0,1.0,  -45},  
          {TURN_ | Flag::END_ANG2,30, 0, 0,0,0, 0,0, 30-180},
          {TURN_ | Flag::END_ANG2,50, 0, 0,0,0, 0,0, 100-180},
          {TURN_ | Flag::END_ANG2,30, 0, 0,0,0, 0,0, -5},
         // ゴール後
          {RESET_LENGTH_,0,0, 0,0,0 ,0,0 , 0},
          {LINE_ | Flag::END_LEN,35, 0.0,    kp*0.45, ki*0, kd*0.6   , 1.0,1.0,  50},  
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 0.68,1.0,  150+50},  //左
          {LINE_ | Flag::END_LEN,20, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0,  150+50+10},  // 直線
           {PARAM_END,0,0,0,0,0,0,0}
        };

    /* ショートカットR用 */
      CParam sc_r1[50] = {

         /** Rコース  **/
         
         {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd , 1.67 ,1.0,  138.0},  //右
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線
          
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右
          {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  275.0},  //直線 減速
          
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd, 1.6,1.0,  325.0},  //右 S
          {LINE_ | Flag::END_LEN,43, 0.0,   kp, ki, kd  , 1.0,1.0,  345.0},  //
          {LINE_ | Flag::END_LEN,45, 0.0, kp, ki, kd , 0.645 ,1.0,  473.0},  //左大

          {LINE_ | Flag::END_LEN,46, 0.0, kp, ki, kd , 1.0,1.0,  515.0},  // 直線
         {LINE_ | Flag::END_LEN,50, 0.0,    kp*1.4, ki*1.4, kd*1.4  , 0.8 ,1.0,  581.0},  // 左
        {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd   , 1.0,1.0,  650.0},  //直線  バックストレート
         {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki, kd  , 1.0,1.0,  665.0},  //直線  バックストレート　減速
       {LINE_ | Flag::END_LEN,45, 0.0,     kp, ki, kd  , 1/1.2,1.0,  720.0},  //左
          {LINE_ | Flag::END_LEN,45, 0.0,  kp, ki, kd  , 1.0,1.0,  745.0},  // 直線真ん中

           {STRAIGHT_ | Flag::END_LEN,45, 0.0,  0,0, 0 , 1.0,1.0,  780.0},  
          {VIRTUAL_ | Flag::END_LEN,45, 18,  kp, ki, kd  , 1.0,1.0,  840},  
    
         // ゴール後
        {LINE_ | Flag::END_LEN,45, 0.0,    kp, ki*0.0, kd , 1.0,1.0,  900.0},  // 直線
        {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd , 1.0,1.0,  1000.0},  // 直線
          //{LINE_ | Flag::END_LEN,40, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1170.0},  //右
          {LINE_ | Flag::END_LEN,35, 0.0,    kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  1045.0},  //右
        //{LINE_ | Flag::END_LEN,20, 0.0,    kp*0.25, ki*0.0, kd*0.1 , 1.0,1.0,  1190.0},  // 直線
        {LINE_ | Flag::END_LEN,10, 0.0,   8.5, 2.0, 2.1  , 1.0,1.0,  1080.0},  // 直線？
          {PARAM_END, 0,0,0,0,0,0,0}
        };

        /*************************/
        /* Rコース                */ 
        /* ショートカット ライントレース無し れんこん */
        /*************************/
        CParam sc_r2[50] = {
         /** Rコース **/        

          {LINE_ | Flag::END_LEN,55, 0.0,  kp, ki, kd , 1.0,1.0,  75.0},  // 直線
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd , 1.67 ,1.0,  138.0},  //右
          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd, 1.0,1.0,  193.0},  //直線

          {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd,  1/0.7,1.0,  260.0},  //右

          {LINE_ | Flag::END_LEN,52, 0.0,    kp, ki, kd  , 1.0,1.0,  265.0},  //直線 減速
          {RESET_LENGTH_,0,0, 0,0,0, 0,0, 0},
          {VIRTUAL_ | Flag::END_ANG,50, 28,  vkp, vki, vkd  , 1.0,1.0,  1},  
          {VIRTUAL_ | Flag::END_LEN,50, 28,  vkp, vki, vkd  , 1.0,1.0,  65},  
          {STRAIGHT_ | Flag::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+11},  

          {VIRTUAL_ | Flag::END_ANG2,50, -28,  vkp, vki, vkd  , 1.0,1.0,  SOUT_ANGLE},  //LEN  65+10+109
          {STRAIGHT_ | Flag::END_LEN,50, 0.0,  0,0, 0 , 1.0,1.0,  65+10+109+14}, 
          {VIRTUAL_ | Flag::END_ANG2,52, -48, vkp, vki, vkd  , 1.0,1.0,  BACKSTRAIGHT_ANGLE},  
          {RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          //  {LINE_ | Flag::END_LEN,55, 0.0,    kp, ki, kd*1.2   , 1.0,1.0,  80+68},  //直線  バックストレート
          {STRAIGHT_ | Flag::END_LEN,58, 0.0,    0, 0, 0   , 1.0,1.0,  72},  //直線  バックストレート

          //{VIRTUAL_ | Flag::END_LEN,50, -48,  vkp, vki, vkd  , 1.0,1.0,  80+68+103},  
          {VIRTUAL_ | Flag::END_ANG2,55, -48,  vkp, vki, vkd  , 1.0,1.0, LAST_CURVEIN_ANGLE},  
          {RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {STRAIGHT_ | Flag::END_LEN,55, 0.0,  0,0, 0 , 1.0,1.0,  6},
          //  ロゴに侵入
          {VIRTUAL_ | Flag::END_ANG2,50, 26,  vkp, vki, vkd  , 1.0,1.0,  LAST_CURVEOUT_ANGLE},  
          {RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          {LINE_ | Flag::END_LEN,40, 0.0,    kp*0.6, ki*0, kd   , 1.0,1.0,  50},  
          //   {STRAIGHT_ | Flag::END_LEN,0, 0.0,  0,0, 0 , 1.0,1.0,  99999.0},  
          {RESET_LENGTH_, 0, 0, 0,0,0, 0,0, 0},

          // ゴール後
          {LINE_ | Flag::END_LEN,50, 0.0,    kp, ki, kd , 1.0,1.0,  150},  // 直線
          {LINE_ | Flag::END_LEN,30, 0.0,  kp*0.5, ki*0.5, kd*0.5 , 1/0.68,1.0,  150+50},  //右
          {LINE_ | Flag::END_LEN,10, 0.0,    8.5, 2.0, 2.1  , 1.0,1.0, 150+50 +5},  // 直線
          {PARAM_END,0,0,0,0,0,0,0}
        };

        CParam scan[50] = {
           //{LINE_ | Flag::END_LEN,20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
           {STRAIGHT_ | Flag::END_LEN,20, 0.0,  0,0, 0 , 1.0,1.0,  5.0},  //低速スキャン用
          {STRAIGHT_ | Flag::END_LEN,-20, 0.0,  0,0, 0 , 1.0,1.0,  2000.0},  //低速スキャン用
           {PARAM_END,0,0,0,0,0,0,0}
       };



       // 運搬テスト
        CParam carry[50] = {
//{LINE_ | Flag::END_LEN,23, 0.0,  8, 3.05, 1.2 , 1.0,1.0,  200000.0},  
          {LINE_ | Flag::END_LEN,23, 0.0, CARRY_KP, CARRY_KI, CARRY_KD, 1.0,1.0,  200000.0},  
          {PARAM_END,0,0,0,0,0,0,0}
       };



       // 仮想トレーステスト
        CParam vtrace[50] = {
         //  {VIRTUAL_ | Flag::END_LEN,40, -19, kp*1.0, ki*1.0, 5.0, 1.0,1.0,  200000.0},
         // {SET_GOAL_PT_,0,0,0,0,0,0,0,0},
          {VIRTUAL_ | Flag::END_ANG,5, 5, vkp*0.5, vki*0.5, vkd*0.5, 1.0,1.0,  90.0},

          {STRAIGHT_ | Flag::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  


          {LINE_ | Flag::END_LEN,45, 0, kp*1.0, ki*1.0, kd, 1.0,1.0,  50.0}, 
          {STRAIGHT_  | Flag::END_LEN,-45,0, 0,0,0, 0,0, 20.0} ,
       //   {TURN_,20, 0 , 0,0,0 , 1.0,1.0,  185.0},
          {VIRTUAL_ | Flag::END_ANG,-45, -18, vkp*0.5, vki*0.5, vkd*0.5, 1.0,1.0,  180.0},

          {VIRTUAL_ | Flag::END_ANG,-45, 18, vkp*1.0, vki*1.0, vkd, 1.0,1.0,  -180.0},
          {STRAIGHT_ | Flag::END_LEN,0, 0, 0,0,0,  1.0,1.0,  99999.0},  

           {PARAM_END,0,0,0,0,0,0,0}
       };



       // モーターテスト
        CParam motor[50] = {
           {LINE_ | Flag::END_LEN,100, 0.0,  0, 0, 0 , 1.0,1.0,  200000.0},  
           {PARAM_END,0,0,0,0,0,0,0}
       };



        int param_idx=0;
        int param_select;

        SimpleWalker *mWk[10];
        SimpleWalker *mActive;

};

#endif