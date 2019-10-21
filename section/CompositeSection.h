#ifndef _COMPOSITE_SECTION_H_
#define _COMPOSITE_SECTION_H_

#include "LineTracer.h"
#include "SDFile.h"
#include "Section.h"
#include "Judge.h"
#include "Turn.h"
#include "StraightWalker.h"
#include "VirtualTracer.h"
#include "ArmControl.h"
#include "ParamStruct.h"
#include "BingoEnum.h"
#include "Flag.h"
#include "Const.h"

//#include "SectionCreate.h"

/*
typedef struct _PARAM_S {
    double fwd;
    double target;
    double kp;
    double ki;
    double kd;
    double curve;
    double ckp;

    double len;
} SParam;

typedef struct _PARAM_P {
    double fwd;
    double target;
    double len;
    double turn;
    Flag::Method runFlag;
    Flag::End endFlag;
} PParam;
*/

class CompositeSection : public Section {
    public:
        CompositeSection(Judge *judge,
                         LineTracer *tracer,
                         StraightWalker *straight,
                         Turn *turn,
                         VirtualTracer *vt,
                         ArmControl *arm,
                         SDFile *sdfi1le);
        ~CompositeSection();
        bool run();
        void execUndefined();
        void execInit();
        void execRun();
        void execNextParam();
        void setParamL(float fwd, float border);
        void setSection(PParam p, int len);
        void setActionUndefined();
        void setParamVirtual(double fwd, double center);

        void saveHSV();
        hsv_t getHSV();
        hsv_t getHSV(int i);

        /*
        void setAction(int action);
        */


    private:
        enum State {
            EXEC_UNDEF,
            EXEC_INIT,
            EXEC_RUN,
            EXEC_END
        };

        ArmControl *mArm;
        SDFile *mSd;
        State mState;
        SParam mTmpS;
        PParam mTmpP;
       // LineTracer *mLineTracer;
        //SectionCreate *mSC;

        Flag::Method mAction;
        int param_idx=0;

        int hsv_idx=0;        //HSV値仮保存用インデックス
        hsv_t hsvArray[100];  //HSV値仮保存用
        int save_idx=0;       //HSV値保存用インデックス
        hsv_t mHsv[10];       //HSV値保存用

        PParam* mParam;   //double fwd ,double target ,double len ,double turn ,Method runFlag ,End endFlag

        //PParam* mParamP;      //double fwd ,double target ,double len ,double turn ,Method runFlag ,End endFlag
        
        PParam mParamP[50] = {      //double fwd ,double target ,double len ,double turn ,Method runFlag ,End endFlag
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{-22.0d ,0.0d                   ,-3     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          //{ 0.0d  ,0.0d                   , 0     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //停止処理

          //{0.0d   ,50.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作
          //{0.0d   ,25.0d                  ,0      ,0  ,Flag::RUN_COLOR    ,Flag::END_CNT },  //一定時間色取得
          //{0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_SAVE     ,Flag::END_ALL },  //ブロック色保存
          
          //{0.0d   ,60.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作
          //{0.0d   ,15.0d                  ,0      ,0  ,Flag::RUN_COLOR    ,Flag::END_CNT },  //一定時間色取得
          //{0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_SAVE     ,Flag::END_ALL },  //ブロック色保存

          //{0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{ 0.0d  ,  0.0d                 ,0      , 0 ,Flag::RUN_TURN     ,Flag::END_ALL },  //スピードコントロール解除
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{ 22.0d ,0.0d                   , 3     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで前進
          
          //{0.0d   ,70.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作
          //{0.0d   ,25.0d                  ,0      ,0  ,Flag::RUN_COLOR    ,Flag::END_CNT },  //一定時間色取得
          //{0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_SAVE     ,Flag::END_ALL },  //ブロック色保存

          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{ 1.0d  ,-89.0d                 ,0      ,10,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
         // { 1.0d  ,-89.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°左旋回⓶
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 0.0d  ,0.0d                   ,10     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理


          /*  //ビンゴゾーン進入
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {25.0d  ,0.0d                   ,10     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          */

          /*  //5-8間移動
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          */
          /*  //0-7間移動
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          */

          /*  //右45°旋回からスローイン
          {25.0d  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  , 30.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持45°右旋回⓵
          { 1.0d  , 44.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック保持45°右旋回⓶
          { 0.0d  ,  0.0d                 ,0      ,0  ,Flag::RUN_TURN     ,Flag::END_ALL },  //旋回停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 80.0d ,80.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ARM },  //ブロックをスローイン
          { 0.0d  , 0.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //スローイン後にアームを既定の位置に戻す
          
          { 2.0d  ,-30.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持45°左旋回⓵
          { 1.0d  ,-44.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック保持45°左旋回⓶
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          */

          /*  //左45°旋回からスローイン
          {25.0d  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-30.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持45°左旋回⓵
          { 1.0d  ,-44.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック保持45°左旋回⓶
          { 0.0d  ,  0.0d                 ,0      ,0  ,Flag::RUN_TURN     ,Flag::END_ALL },  //旋回停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 80.0d ,80.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ARM },  //ブロックをスローイン
          { 0.0d  , 0.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //スローイン後にアームを既定の位置に戻す
          
          { 2.0d  ,-30.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持45°左旋回⓵
          { 1.0d  ,-44.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック保持45°左旋回⓶
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          */
         
          /*  //4-6間移動
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          */
        /*  
            //4-6間からガレージイン（R)
          {20.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 0.0d  , 45.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し45°右旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {20.0d  ,0.0d                   ,15     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_BLK },  //黒線を検知するまで直進
          { 0.0d  ,-45.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し45°左旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更

          {20.0d  ,(double)COLOR::BLUE    ,25     ,0  ,Flag::RUN_ONLINE   ,Flag::END_COL },  //既定の色まで黒線寄りにライントレース
          {20.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          
          { 1.0d  ,-75.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
          { 1.0d  ,-89.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°左旋回⓶
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,40     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
        */  

          
 
          /*
          {20.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {15.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {-20.0d ,0.0d                   ,-15    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 0.0d  , 86.0d                 ,0      ,1  ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施

          {25.0d  ,0.0d                   ,27     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_BLK },  //黒線を検知するまで直進
          {25.0d  ,0.0d                   ,1      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
           
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回 
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {-15.0d ,0.0d                   ,-15    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {15.0d  ,0.0d                   ,12     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {15.0d  ,(double)COLOR::RED     ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          */


          /*  //黒ブロック回収
          {20.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {15.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {-20.0d ,0.0d                   ,-15    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 0.0d  ,-88.0d                 ,0      ,1  ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施

          {25.0d  ,0.0d                   ,25     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_BLK },  //黒線を検知するまで直進
          {25.0d  ,0.0d                   ,1      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          
          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {-15.0d ,0.0d                   ,-15    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {15.0d  ,0.0d                   ,12     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {15.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          */

          //{100.0d ,80.0d                ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ARM },  //ブロックをスローイン
          //{ 0.0d  , 0.0d                ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //スローイン後にアームを既定の位置に戻す

          /*
          {25.0d  ,0.0d                   ,25     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進

          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          //{ 0.0d  ,-90.0d               ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,20     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          //{ 0.0d  ,-90.0d               ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,20     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,20     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進

          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::BLUE    ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::RED     ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::YELLOW  ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,22     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,-90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,0.0d                   ,20     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進

          {25.0d  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          */


          //{15.0d  ,-90.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          //{ 0.0d  ,-90.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
        
          //{0.0d   ,0.0d                   ,1000   ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //モーター停止処理
          //{ 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
      
          /*  //テンプレ
          {25.0d  ,0.0d                   ,20     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {25.0d  ,0.0d                   ,25     ,0  ,Flag::RUN_ONLINE   ,Flag::END_LEN },  //一定距離まで黒線寄りにライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {25.0d  ,(double)COLOR::GREEN   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {25.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_BLK },  //黒線を検知するまで直進
          {25.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {-25.0d ,0.0d                   ,-25    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 1.0d  ,-75.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
          { 1.0d  ,-89.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°左旋回⓶
          {0      ,Turn::RIGHT            ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを右に変更
          {0.0d   ,0.0d                   ,1000   ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //モーター停止処理
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
          */



          /*  //旧
          {15.0d  ,-90.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°左旋回
          { 0.0d  ,-90.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回
          { 2.0d  , 90.0d                 ,0      ,10 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回

          /* //交点サークルの外から次の交点のブロックを拾って左に90°旋回
          {25,0,20,0,LINETRACE},
          {0,0,0,0,RECORD},
          {25,(double)COLOR::GREEN,0,0,STRAIGHT},
          {25,0,5,0,STRAIGHT},
          {15.0d,-90,0,20,TURNING},//ブロック保持90°左旋回
          */
          
          //{0,0,0,0,RECORD},//レコードカウント実施
          //{0,Turn::RIGHT,0,0,EDGE},//ライントレース走行用エッジを右に変更
          //{0,Turn::LEFT,0,0,EDGE},//ライントレース走行用エッジを左に変更
          //{2.5d,180,0,12,TURNING},//ブロック保持180°右旋回、約2秒
          //{15.0d,90,0,20,TURNING},//ブロック保持90°右旋回、約1秒
          //{15.0d,-90,0,20,TURNING},//ブロック保持90°左旋回
          //{25,(double)COLOR::YELLOW,0,0,LINETRACE},//黄色を見るまでライントレース
          //{25,(double)COLOR::YELLOW,0,0,STRAIGHT},//黄色を見るまで直進
          //{25,0,20,0,STRAIGHT},//直進
          //{25,0,35,0,LINETRACE},//ライントレース

          /*          
          {0,0,0,0,RECORD},
          {0,0,1000,0,STRAIGHT},
          {0,0,0,0,ENDACTION}
          */
        
        };
        

        SParam mParamS[50] = {
          {Flag::RUN_END,0,0,0,0,0,0}//終了
        };
         // {20, 0.0,  16, 3.05, 1.1 , 1.0,1.0,  200000.0},  //低速スキャン用
         //  {60, 0.0,  32, 0.5 , 2.8 , 1.0,1.0,  200000.0},  //直線調整用

         /** Lコース **/
         /*
         {65, 0.0,  45, 2.05, 4.0 , 1.0,1.0,  75.0},  // 直線
          {50, 0.0,    45, 2.05, 4.0 , 0.68,1.0,  137.0},  //左

          {60, 0.0,    45, 2.05, 4.0, 1.0,1.0,  193.0},  //直線
          {50, 0.0,    45, 2.05, 4.0,  0.7,1.0,  265.0},  //左

         {50, 0.0,    45, 2.05, 4.0  , 1.0,1.0,  278.0},  //直線
          {45, 0.0,    45, 2.05, 4.0  , 0.6,1.0,  328.0},  //左

          {45, 0.0,   45, 2.05, 4.0  , 0.99,1.0,  358.0},  //

          {55, 0.0,   45, 2.05, 5.0 , 1.35,1.0,  480.0},  //右

          {65, 0.0,  45, 2.05, 5.0 , 1.0,1.0,  515.0},  // 直線
         {55, 0.0,   45, 2.05, 5.0 , 1.2,1.0,  581.0},  // 右
        {60, 0.0,    45, 2.05, 4.0  , 1.0,1.0,  665.0},  //直線
        {50, 0.0,    45, 2.05, 4.0  , 1.2,1.0,  720.0},  //右
          {50, 0.0,    45, 2.05, 4.0 , 1.0,1.0,  755.0},  //
          {50, 0.0,    45, 2.05, 4.0 , 1.3,1.0,  775.0},  //右

          {50, 0.0,     45, 2.05, 4.0 , 0.68 ,1.0,  915.0},  //左
        {65, 0.0,    45, 2.05, 4.0 , 1.0,1.0,  980.0},  // 直線
                {0,0, 0,0,0, 0,1.0 ,10000},
    
         // ゴール後
        {65, 0.0,    45, 2.05, 5.0 , 1.0,1.0,  1130.0},  // 直線
          {40, 0.0,    45, 2.05, 2.5 , 0.68,1.0,  1170.0},  //左
        {20, 0.0,    45, 2.05, 2.5 , 1.0,1.0,  1190.0},  // 直線
            {0,0, 0,0,0, 0,1.0 ,10000},
        */

          /** Rコース  **/
          /*
          {65, 0.0,  45, 2.05, 4.0 , 1.0,1.0,  75.0},  // 直線
          {50, 0.0,    45, 2.05, 4.0 , 1/0.68,1.0,  137.0},  //左

          {60, 0.0,    45, 2.05, 4.0, 1.0,1.0,  193.0},  //直線
          {50, 0.0,    45, 2.05, 4.0,  1/0.7,1.0,  265.0},  //左

          {50, 0.0,    45, 2.05, 4.0  , 1.0,1.0,  278.0},  //直線
          {45, 0.0,    45, 2.05, 4.0  , 1/0.6,1.0,  328.0},  //左

          {45, 0.0,   45, 2.05, 4.0  , 1/0.99,1.0,  358.0},  //

          {55, 0.0,   45, 2.05, 4.0 , 1/1.35,1.0,  480.0},  //右

          {65, 0.0,  45, 2.05, 4.0 , 1.0,1.0,  515.0},  // 直線
          {55, 0.0,   45, 2.05, 4.0 , 1/1.2,1.0,  581.0},  // 右
          {60, 0.0,    45, 2.05, 4.0  , 1.0,1.0,  665.0},  //直線
          {50, 0.0,    45, 2.05, 4.0  , 1/1.2,1.0,  720.0},  //右
          {50, 0.0,    45, 2.05, 4.0 , 1.0,1.0,  755.0},  //
          {50, 0.0,    45, 2.05, 4.0 , 1/1.3,1.0,  775.0},  //右

          {50, 0.0,     45, 2.05, 4.0 , 1/0.68 ,1.0,  915.0},  //左
          {65, 0.0,    45, 2.05, 4.0 , 1.0,1.0,  980.0},  // 直線
    
          // ゴール後
          {65, 0.0,    45, 2.05, 4.0 , 1.0,1.0,  1130.0},  // 直線
          {40, 0.0,    45, 2.05, 2.5 , 1/0.68,1.0,  1170.0},  //左
          {20, 0.0,    45, 2.05, 2.5 , 1.0,1.0,  1190.0},  // 直線
          {0,0, 0,0,0, 0,1.0 ,10000},
          */

          //{ENDACTION,0,0,0,0,0,0}//終了
        //};

};

#endif